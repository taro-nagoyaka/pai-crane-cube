"""Local checks without ROS / PyTorch. Run: python3 -m unittest -v test_core.py"""
import ast
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from common import JOINTS, FPS, IMAGE_SIZE, LOWER, UPPER, reorder, bounded_target, image_rgb
from convert_dataset import validate
from prepare_scene import build_launch, build_world
from prepare_teacher import patch_launch
from teacher_reference import sample_points, combined_reference
from policy_server import configure_action_steps, configure_temporal_ensemble
from replay_teacher import load_episode


class CoreTests(unittest.TestCase):
    def test_replay_loads_actions_instead_of_states(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'episode.npz'
            states = np.zeros((3, 5), dtype=np.float32)
            actions = np.full((3, 5), 0.01, dtype=np.float32)
            fields = dict(joint_names=np.array(JOINTS), fps=np.array(FPS),
                          success=np.array(True), state=states, action=actions)
            np.savez(path, **fields)
            initial, loaded = load_episode(path)
            np.testing.assert_allclose(initial, states[0])
            np.testing.assert_allclose(loaded, actions)
            self.assertFalse(np.array_equal(loaded, states))
            fields['action'][1, 0] = 100
            np.savez(path, **fields)
            with self.assertRaises(ValueError):
                load_episode(path)

    def test_temporal_ensemble_configuration(self):
        calls = []
        class Policy:
            def __init__(self):
                self.config = SimpleNamespace(chunk_size=10, n_action_steps=10, temporal_ensemble_coeff=None)
                self.resets = 0
            def reset(self):
                self.resets += 1
        policy = Policy()
        def factory(coefficient, chunk_size):
            calls.append((coefficient, chunk_size))
            return 'ensemble'
        configure_temporal_ensemble(policy, 0.01, factory)
        self.assertEqual(policy.config.n_action_steps, 1)
        self.assertEqual(policy.temporal_ensembler, 'ensemble')
        self.assertEqual(calls, [(0.01, 10)])
        self.assertEqual(policy.resets, 1)
        for invalid in (-1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                configure_temporal_ensemble(policy, invalid, factory)

    def test_action_steps_validation_and_reset(self):
        class Policy:
            def __init__(self):
                self.config = SimpleNamespace(chunk_size=10, n_action_steps=1, temporal_ensemble_coeff=None)
                self.resets = 0
            def reset(self):
                self.resets += 1
        policy = Policy()
        configure_action_steps(policy, 10)
        self.assertEqual(policy.config.n_action_steps, 10)
        self.assertEqual(policy.resets, 1)
        configure_action_steps(policy, None)
        self.assertEqual(policy.resets, 1)
        for invalid in (0, 11, 2.5):
            with self.assertRaises(ValueError):
                configure_action_steps(policy, invalid)
        policy.config.temporal_ensemble_coeff = 0.01
        with self.assertRaises(ValueError):
            configure_action_steps(policy, 10)

    def test_teacher_reference_interpolation_and_group_order(self):
        np.testing.assert_allclose(sample_points([0, 2], [[0, 2], [2, 4]], 1), [1, 3])
        np.testing.assert_allclose(sample_points([0, 2], [[0, 2], [2, 4]], 5), [2, 4])
        refs = [([JOINTS[2], JOINTS[0]], 10, [0, 2], [[0, 0], [2, 1]]),
                ([JOINTS[4]], 10, [0, 2], [[-0.5], [0.1]])]
        q = combined_reference([0, 0.2, 0, 0.4, 0], refs, 10.8)
        np.testing.assert_allclose(q, [0.5, 0.2, 1, 0.4, -0.2])
        with self.assertRaises(ValueError):
            sample_points([1, 0], [[0], [1]], 0.5)

    def test_teacher_launch_only_replaces_example_node(self):
        source = '''def generate_launch_description():
    node = Node(name=[LaunchConfiguration('example'), '_node'],
       package='crane_plus_examples', executable=LaunchConfiguration('example'),
       parameters=[{'robot_description': description_loader.load()}, semantic, kinematics])
    return LaunchDescription([node])
'''
        patched = patch_launch(source)
        ast.parse(patched)
        self.assertIn("package='crane_cube_teacher'", patched)
        self.assertIn("executable='lift_teacher'", patched)
        self.assertIn('description_loader.load()', patched)
        with self.assertRaises(ValueError):
            patch_launch('x = 1')

    def test_joint_order(self):
        names = [JOINTS[0], JOINTS[1], JOINTS[3], JOINTS[2], JOINTS[4]]
        np.testing.assert_array_equal(reorder(names, [1, 2, 4, 3, 5]), [1, 2, 3, 4, 5])
        with self.assertRaises(KeyError):
            reorder(names[:-1], [1, 2, 4, 3])

    def test_action_limits(self):
        np.testing.assert_allclose(bounded_target([100] * 5), UPPER)
        np.testing.assert_allclose(bounded_target([-100] * 5), LOWER)
        with self.assertRaises(ValueError):
            bounded_target([0, 0, float('nan'), 0, 0])

    def test_image_row_padding_and_bgr(self):
        # Two BGR red pixels plus two padding bytes per row.
        data = bytes([0, 0, 255, 0, 0, 255, 17, 17] * 2)
        msg = SimpleNamespace(encoding='bgr8', height=2, width=2, step=8, data=data)
        rgb = image_rgb(msg)
        self.assertEqual(rgb.shape, (IMAGE_SIZE, IMAGE_SIZE, 3))
        np.testing.assert_array_equal(rgb[0, 0], [255, 0, 0])

    def test_scene_preserves_cube_pose(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'table.sdf'
            p.write_text('''<sdf version="1.6"><world name="default">
              <physics name="p" type="ignored"><real_time_factor>1</real_time_factor></physics>
              <include><uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Wood%20cube%205cm</uri>
              <pose>0 -0.1 1.0 0 0 0</pose></include></world></sdf>''')
            root = build_world(p, 0.25).getroot()
            world = root.find('world')
            self.assertEqual(world.findtext('include/pose'), '0 -0.1 1.0 0 0 0')
            self.assertEqual(world.findtext('include/name'), 'woodcube')
            self.assertEqual(world.findtext('physics/real_time_factor'), '0.25')
            self.assertEqual(world.findtext('model/link/sensor/topic'), '/camera/front/image')
            self.assertEqual(len(world.findall('plugin')), 1)

    def test_launch_patch_keeps_clock_and_spawn(self):
        src = '''import os
def generate_launch_description():
    world_file = os.path.join(get_package_share_directory('crane_plus_gazebo'), 'worlds', 'table.sdf')
    gazebo_spawn_entity = Node(arguments=['-name', 'crane_plus', '-z', '1.015'])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'])
    return LaunchDescription([gazebo_spawn_entity, bridge])
'''
        patched = build_launch(src)
        ast.parse(patched)
        self.assertIn('1.015', patched)
        self.assertIn('/clock@', patched)
        self.assertIn('/camera/front/image@', patched)
        self.assertIn('table_camera.sdf', patched)
        with self.assertRaises(ValueError):
            build_launch('world_file = "test"')

    def test_episode_validation_rejects_wrong_order_and_time_gaps(self):
        n = FPS * 3
        episode = dict(state=np.zeros((n, 5), dtype=np.float32),
                       action=np.zeros((n, 5), dtype=np.float32),
                       image=np.zeros((n, IMAGE_SIZE, IMAGE_SIZE, 3), dtype=np.uint8),
                       time=np.arange(n) / FPS, fps=np.array(FPS),
                       joint_names=np.array(JOINTS), success=np.array(True))
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'episode.npz'
            np.savez_compressed(p, **episode)
            self.assertEqual(validate(p), n)
            episode['joint_names'] = episode['joint_names'][::-1]
            np.savez_compressed(p, **episode)
            with self.assertRaises(ValueError):
                validate(p)
            episode['joint_names'] = np.array(JOINTS)
            episode['time'][10:] += 0.5
            np.savez_compressed(p, **episode)
            with self.assertRaises(ValueError):
                validate(p)


if __name__ == '__main__':
    unittest.main()
