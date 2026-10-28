"""Batch data integrity and physical-screening tests; no ROS installation needed."""
import ast
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from batch_support import (PoseTextParser, make_trial, trial_environment, trial_settings,
                           verify_episode)
from collect_batch import count_success
from batch_archive import save_trial


class BatchTests(unittest.TestCase):
    def test_pose_parser_uses_only_named_model_and_header_time(self):
        found = []
        parser = PoseTextParser(lambda t, xyz: found.append((t, xyz)))
        text = '''header {
  stamp {
    sec: 12
    nsec: 300000000
  }
}
pose {
  name: "link"
  position {
    x: 999
  }
}
pose {
  name: "woodcube"
  id: 42
  position {
    x: -1.0e-3
    y: -0.1
    z: 1.3
  }
  orientation {
    w: 1
  }
}
'''
        for line in text.splitlines(keepends=True):
            parser.feed(line)
        self.assertEqual(len(found), 1)
        self.assertAlmostEqual(found[0][0], 12.3)
        np.testing.assert_allclose(found[0][1], [-.001, -.1, 1.3])

    def episode(self):
        t = np.arange(60) / 10
        q = np.zeros((60, 5)); q[:, 4] = .10
        a = q.copy(); a[:, 4] = .174
        xyz = np.tile([0., 0., 1.31], (60, 1))
        stages = ['stage:grasp'] * 20 + ['stage:hold_two_sim_seconds'] * 40
        return dict(times=t, states=q, actions=a, cubes=xyz, stages=stages,
                    initial_cube=np.array([0., -.1, 1.025]),
                    joint_stamps=t+10, image_stamps=t+9.98)

    def test_success_retains_cube_above_wrist_upright(self):
        self.assertTrue(verify_episode(**self.episode())['passed'])

    def test_confirmed_14cm_wrist_extension_is_not_a_failed_grasp(self):
        ep = self.episode()
        # Regression for the operator-confirmed result: 14.2088 cm from WRIST,
        # 40.3357 cm rise, upright arm. This point is beyond the old 10 cm cutoff.
        ep['cubes'][:, 2] = 1.2629 + .1420880340837143
        ep['initial_cube'][2] = ep['cubes'][0, 2] - .4033570093591974
        result = verify_episode(**ep)
        self.assertTrue(result['passed'], result)
        self.assertAlmostEqual(result['metrics']['max_cube_wrist_distance_m'], .1420880340837143)
        self.assertEqual(result['verification_version'], 2)

    def test_long_hand_envelope_does_not_allow_sideways_or_far_cubes(self):
        ep = self.episode(); ep['cubes'][:, 0] = .08
        self.assertIn('cube_too_far_sideways', verify_episode(**ep)['reasons'])
        ep = self.episode(); ep['cubes'][:, 2] = 1.2629 + .181
        self.assertIn('cube_not_near_arm', verify_episode(**ep)['reasons'])

    def test_rejected_arrays_are_retained_and_cannot_enter_training(self):
        from common import IMAGE_SIZE
        from convert_dataset import validate
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); out, rejected = root/'raw', root/'logs/trial_00000'
            ep = self.episode()
            samples = [(t, q, np.zeros((IMAGE_SIZE, IMAGE_SIZE, 3), dtype=np.uint8), a, js, ims)
                       for t, q, a, js, ims in zip(ep['times'], ep['states'], ep['actions'],
                                                 ep['joint_stamps'], ep['image_stamps'])]
            report = dict(passed=False, reasons=['example_rejection'])
            path = save_trial(out, rejected, samples, ep['cubes'], ep['times'], ep['stages'],
                              ep['initial_cube'], {}, report)
            self.assertEqual(path.parent, rejected)
            self.assertFalse(out.exists())
            with np.load(path, allow_pickle=False) as d:
                self.assertFalse(bool(d['success']))
                self.assertEqual(d['image'].shape[0], len(samples))
                np.testing.assert_allclose(d['action'], ep['actions'])
            with self.assertRaises(ValueError): validate(path)
            success_path = save_trial(out, rejected, samples, ep['cubes'], ep['times'], ep['stages'],
                                      ep['initial_cube'], {}, dict(passed=True, reasons=[]))
            self.assertEqual(success_path.parent, out)
            self.assertEqual(validate(success_path), len(samples))

    def test_static_cube_on_table_cannot_be_success(self):
        ep = self.episode(); ep['cubes'][:, 2] = 1.025
        result = verify_episode(**ep)
        self.assertFalse(result['passed'])
        self.assertIn('cube_not_lifted_15cm', result['reasons'])

    def test_dropped_drifting_far_or_low_cube_rejected(self):
        for change in ('drop', 'drift', 'far', 'below_wrist'):
            ep = self.episode()
            if change == 'drop': ep['cubes'][-5:, 2] = 1.03
            if change == 'drift': ep['cubes'][-1, 0] = .04
            if change == 'far': ep['cubes'][:, 0] = .5
            if change == 'below_wrist': ep['cubes'][:, 2] = 1.22
            self.assertFalse(verify_episode(**ep)['passed'], change)

    def test_upright_alone_and_grasp_without_hold_are_not_success(self):
        ep = self.episode(); ep['states'][:, 2] = -.3
        self.assertIn('arm_not_upright', verify_episode(**ep)['reasons'])
        ep = self.episode(); ep['stages'] = ['stage:grasp'] * 60
        self.assertFalse(verify_episode(**ep)['passed'])

    def test_invalid_recording_or_jump_not_trainable(self):
        ep = self.episode(); ep['image_stamps'][-1] = ep['image_stamps'][-2]
        self.assertIn('image_joint_timing', verify_episode(**ep)['reasons'])
        ep = self.episode(); ep['actions'][10, 0] = .2
        self.assertIn('reference_jump_over_deploy_limit', verify_episode(**ep)['reasons'])
        ep = self.episode(); ep['states'][0, 0] = np.nan
        self.assertFalse(verify_episode(**ep)['passed'])

    def test_trials_are_reproducible_varied_and_bounded(self):
        profiles = set()
        settings = [trial_settings(25, i) for i in range(48)]
        for i, s in enumerate(settings):
            self.assertEqual(s, trial_settings(25, i)); profiles.add(s['profile'])
            self.assertLessEqual(abs(s['dx']), .004)
            self.assertLessEqual(abs(s['dy']), .004)
            self.assertLessEqual(abs(s['home_offset']), .06)
            self.assertLessEqual(abs(s['lift_pitch']), .10)
            self.assertEqual(trial_environment(s)['CUBE_TEACHER_HOLD_SECONDS'], '3.0')
        self.assertEqual(len(profiles), 4)
        self.assertTrue(any(s['dx'] != 0 for s in settings))

    def test_trial_scene_and_teacher_offsets_agree_without_editing_source(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            world = root / 'original.sdf'; launch = root / 'original.launch.py'
            source = '''<sdf version="1.6"><world name="default"><include>
              <uri>cube</uri><name>woodcube</name><pose>0 -0.1 1.0 0 0 0</pose>
              </include></world></sdf>'''
            world.write_text(source); launch.write_text("world_file = 'old.sdf'\n")
            s = trial_settings(25, 10)
            make_trial(world, launch, root/'new.sdf', root/'new.launch.py', s)
            xyz = list(map(float, ET.parse(root/'new.sdf').findtext('.//include/pose').split()))
            np.testing.assert_allclose(xyz[:3], [s['dx'], -.1+s['dy'], 1.0])
            self.assertEqual(world.read_text(), source)
            ast.parse((root/'new.launch.py').read_text())
            self.assertIn(str(root/'new.sdf'), (root/'new.launch.py').read_text())

    def test_resume_counts_only_verified_and_never_counts_logs(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); (root/'batch_logs').mkdir()
            np.savez(root/'episode_one.npz', success=True,
                     automatic_verification=np.array(json.dumps({'passed': True})))
            self.assertEqual(count_success(root), 1)
            np.savez(root/'episode_bad.npz', success=True,
                     automatic_verification=np.array(json.dumps({'passed': False})))
            with self.assertRaises(ValueError): count_success(root)


if __name__ == '__main__':
    unittest.main()
