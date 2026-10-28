"""Pure helpers for varied demonstrations and conservative success screening."""
import ast
import re
import xml.etree.ElementTree as ET

import numpy as np


def trial_settings(seed, index):
    """Deterministic strata: nominal, near-home, tilted lift, and both."""
    rng = np.random.default_rng(np.random.SeedSequence([seed, index]))
    profile = ('nominal', 'near_home', 'lift_recovery', 'both')[index % 4]
    # Start each stratum at the proven cube position. Later trials add <=4 mm offsets.
    dx, dy = (rng.uniform(-0.004, 0.004, 2) if index >= 4 else (0., 0.))
    home = float(rng.choice([-1, 1]) * rng.uniform(0.025, 0.06)) if profile in ('near_home', 'both') else 0.
    pitch = float(rng.choice([-1, 1]) * rng.uniform(0.04, 0.10)) if profile in ('lift_recovery', 'both') else 0.
    return dict(seed=seed, index=index, profile=profile, dx=float(dx), dy=float(dy),
                home_offset=home, lift_pitch=pitch)


def trial_environment(settings):
    return {key: str(value) for key, value in {
        'CUBE_TEACHER_DX': settings['dx'], 'CUBE_TEACHER_DY': settings['dy'],
        'CUBE_TEACHER_HOME_OFFSET': settings['home_offset'],
        'CUBE_TEACHER_LIFT_PITCH': settings['lift_pitch'],
        'CUBE_TEACHER_HOLD_SECONDS': 3.0,
    }.items()}


def make_trial(world_path, launch_path, out_world, out_launch, settings):
    tree = ET.parse(world_path)
    world = tree.getroot().find('world')
    if world is None or world.get('name') != 'default':
        raise ValueError('Expected the default CRANE+ world')
    cubes = [inc for inc in world.findall('include') if inc.findtext('name') == 'woodcube']
    if len(cubes) != 1:
        raise ValueError('Expected exactly one named woodcube include')
    pose = cubes[0].find('pose')
    values = list(map(float, pose.text.split()))
    if len(values) != 6 or not np.allclose(values[:2], [0, -0.1]):
        raise ValueError('Original cube position differs from the supported scene')
    values[0] += settings['dx']
    values[1] += settings['dy']
    pose.text = ' '.join(map(str, values))
    ET.indent(tree, space='  ')
    tree.write(out_world, encoding='utf-8', xml_declaration=True)
    launch = ast.parse(launch_path.read_text())
    changed = 0
    for node in ast.walk(launch):
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'world_file' for t in node.targets):
            node.value = ast.Constant(str(out_world.resolve()))
            changed += 1
    if changed != 1:
        raise ValueError('Cannot find one world_file assignment in the camera launch')
    ast.fix_missing_locations(launch)
    out_launch.write_text('# Generated batch scene; original packages unchanged.\n' + ast.unparse(launch) + '\n')


def _field(text, name, default=0.):
    m = re.search(r'\b' + re.escape(name) + r'\s*:\s*([-+\d.eE]+)', text)
    return float(m.group(1)) if m else default


class PoseTextParser:
    """Parse top-level protobuf DebugString blocks from `ign topic -e`.

    SceneBroadcaster's top-level model poses are relative to the world.
    Never use an unnamed/link pose or a ROS TF bridge that dropped entity names.
    """
    def __init__(self, callback, model='woodcube'):
        self.callback, self.model = callback, model
        self.block, self.depth, self.kind, self.stamp = [], 0, None, None

    def feed(self, line):
        if not self.depth:
            m = re.match(r'^\s*(header|pose)\s*\{', line)
            if not m:
                return
            self.kind, self.block = m.group(1), []
        self.block.append(line)
        # Braces inside quoted strings are irrelevant to our model; strip strings first.
        plain = re.sub(r'"(?:\\.|[^"\\])*"', '""', line)
        self.depth += plain.count('{') - plain.count('}')
        if self.depth != 0:
            return
        text = ''.join(self.block)
        if self.kind == 'header':
            self.stamp = _field(text, 'sec') + _field(text, 'nsec') * 1e-9
        else:
            name = re.search(r'\bname\s*:\s*"([^"\\]*)"', text)
            position = re.search(r'\bposition\s*\{([^{}]*)\}', text, re.S)
            if name and name.group(1) == self.model and position and self.stamp is not None:
                xyz = np.array([_field(position.group(1), axis) for axis in ('x', 'y', 'z')])
                if np.isfinite(xyz).all():
                    self.callback(self.stamp, xyz)


def wrist_world(q):
    q = np.asarray(q)
    radius = .083 * np.sin(q[..., 1]) + .0935 * np.sin(q[..., 1] - q[..., 2])
    z = 1.015 + .0714 + .083 * np.cos(q[..., 1]) + .0935 * np.cos(q[..., 1] - q[..., 2])
    return np.stack([radius * np.cos(q[..., 0]), radius * np.sin(q[..., 0]), z], axis=-1)


def verify_episode(times, states, actions, cubes, stages, initial_cube, joint_stamps, image_stamps):
    """Screen height/retention/upright and observation quality, not contact forces."""
    t, q, a, xyz = map(np.asarray, (times, states, actions, cubes))
    reasons = []
    from common import LOWER, UPPER
    if len(t) < 30 or q.shape != (len(t), 5) or a.shape != q.shape or xyz.shape != (len(t), 3):
        return {'passed': False, 'reasons': ['invalid_shapes_or_short_episode']}
    if not all(np.isfinite(v).all() for v in (t, q, a, xyz, initial_cube)):
        return {'passed': False, 'reasons': ['nonfinite_values']}
    if not np.allclose(np.diff(t), .1, atol=.035):
        reasons.append('missing_or_irregular_10hz_samples')
    if np.any(q < LOWER - .03) or np.any(q > UPPER + .03) or np.any(a < LOWER) or np.any(a > UPPER):
        reasons.append('joint_limits')
    if np.any(q[:, 1:3] > .03) or np.any(a[:, 1:3] > .03):
        reasons.append('different_elbow_branch')
    jumps = float(np.max(np.abs(np.diff(a, axis=0))))
    if jumps > .15:
        reasons.append('reference_jump_over_deploy_limit')
    js, ims = np.asarray(joint_stamps), np.asarray(image_stamps)
    if np.max(np.abs(js - ims)) > .1 or np.any(np.diff(ims) <= 0):
        reasons.append('image_joint_timing')
    hold = np.array([str(stage).startswith('stage:hold') for stage in stages])
    selected = np.flatnonzero(hold)
    metrics = {'max_reference_jump_rad': jumps}
    if len(selected) < 2 or t[selected[-1]] - t[selected[0]] < 2.0:
        reasons.append('less_than_2_seconds_final_hold')
    else:
        # Use the final continuous 2-second window; all samples must pass.
        mask = hold & (t >= t[selected[-1]] - 2.0)
        held_xyz, held_q = xyz[mask], q[mask]
        rise = float(np.min(held_xyz[:, 2]) - initial_cube[2])
        held_wrist = wrist_world(held_q)
        relative = held_xyz - held_wrist
        distance = float(np.max(np.linalg.norm(relative, axis=1)))
        horizontal = float(np.max(np.linalg.norm(relative[:, :2], axis=1)))
        drift = float(np.max(np.linalg.norm(held_xyz - held_xyz[0], axis=1)))
        upright = float(np.max(np.abs(held_q[:, :4])))
        metrics.update(min_final_cube_rise_m=rise, max_cube_wrist_distance_m=distance,
                       max_cube_wrist_horizontal_distance_m=horizontal,
                       final_cube_drift_m=drift, max_final_arm_angle_rad=upright,
                       final_cube_mean_world_m=held_xyz.mean(axis=0).tolist(),
                       final_wrist_mean_world_m=held_wrist.mean(axis=0).tolist(),
                       mean_cube_minus_wrist_xyz_m=relative.mean(axis=0).tolist())
        if rise < .15:
            reasons.append('cube_not_lifted_15cm')
        # endtip is the wrist, not the finger/cube centre. Visually confirmed
        # CRANE+ lifts measured 0.1421 m here and were incorrectly rejected at .10.
        # Allow the hand's extension while separately constraining lateral offset.
        if distance > .18:
            reasons.append('cube_not_near_arm')
        if horizontal > .06:
            reasons.append('cube_too_far_sideways')
        if np.any(held_xyz[:, 2] < held_wrist[:, 2] - .02):
            reasons.append('cube_below_upright_wrist')
        if drift > .025:
            reasons.append('cube_not_stably_held')
        if upright > .08:
            reasons.append('arm_not_upright')
        if np.any(held_q[:, 4] < -.05):
            reasons.append('gripper_open_at_finish')
    if not any('stage:grasp' == str(stage) for stage in stages):
        reasons.append('missing_grasp_stage')
    return dict(passed=not reasons, reasons=reasons, metrics=metrics, verification_version=2,
                geometry_limits=dict(max_wrist_distance_m=.18, max_horizontal_offset_m=.06,
                                     min_rise_m=.15, max_hold_drift_m=.025,
                                     max_upright_error_rad=.08, min_hold_s=2.0),
                definition='Gazebo model height/proximity/stability + upright arm; not a contact-force measurement')
