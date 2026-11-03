"""ROS-only configuration consistency, independent of hardware and ROS discovery."""
import copy
import tempfile
import unittest
from pathlib import Path
import yaml
from advrf_ros2_motors_controller.robot_config import load_robot


class RobotConfigTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / 'config').mkdir()
        self.hardware = {'joint_map': {7: 'b', 1: 'a'}, 'profiles': {
            'position': {'a': {'gains': [2., 3., 4., 0., 0.]},
                         'b': {'gains': [5., 6., 7., 0., 0.]}},
            'impedance': {'a': {'gains': [1., 2., 3., 4., 5.]},
                          'b': {'gains': [6., 7., 8., 9., 10.]}}}}
        self.write('advrf_hardware.yaml', self.hardware)
        self.write('advrf_controllers.yaml', {})
        self.write('moveit_controllers.yaml', {'moveit_simple_controller_manager': {
            'controller_names': ['arm'], 'arm': {'type': 'FollowJointTrajectory',
            'action_ns': 'follow_joint_trajectory', 'joints': ['a', 'b']}}})
        self.limits = {'joint_limits': {j: {'min_position': -1., 'max_position': 1.,
            'max_velocity': 2., 'max_acceleration': 3.} for j in ['b', 'a']}}
        self.write('joint_limits.yaml', self.limits)

    def write(self, name, value):
        (self.root / 'config' / name).write_text(yaml.safe_dump(value))

    def load(self, **kwargs):
        return load_robot(self.root, '/robot', **kwargs)

    def test_without_urdf_alignment_and_gain_permutation(self):
        robot = self.load()
        cmd = robot['nodes']['joint_command_controller_node']
        power = robot['nodes']['motor_power_node']
        self.assertEqual(cmd['joints'], ['a', 'b'])
        self.assertEqual(cmd['motor_ids'], [1, 7])
        self.assertEqual(cmd['joint_gains'], [2., 4., 0., 0., 3., 5., 7., 0., 0., 6.])
        self.assertEqual(power['start_joint_gains'], [2., 3., 4., 0., 0., 5., 6., 7., 0., 0.])
        self.assertEqual(robot['action_name'], '/robot/arm/follow_joint_trajectory')

    def test_standalone_controller_without_moveit_file(self):
        (self.root / 'config/moveit_controllers.yaml').unlink()
        self.write('controller.yaml', {'controller': {
            'name': 'arm', 'action_ns': 'follow_joint_trajectory', 'joints': ['a', 'b']}})
        robot = self.load()
        self.assertEqual(robot['action_name'], '/robot/arm/follow_joint_trajectory')
        self.assertEqual(robot['moveit'], {})

    def test_impedance_preserves_all_five_values(self):
        robot = self.load(control_mode='impedance')
        self.assertEqual(robot['nodes']['joint_command_controller_node']['joint_gains'],
                         robot['nodes']['motor_power_node']['start_joint_gains'])

    def test_urdf_supplies_bounds_and_rejects_excessive_limits(self):
        path = self.root / 'robot.urdf'
        path.write_text('<robot name="test">' + ''.join(
            f'<joint name="{j}" type="revolute"><limit lower="-1" upper="1" velocity="2"/></joint>'
            for j in ['a', 'b']) + '</robot>')
        for entry in self.limits['joint_limits'].values():
            del entry['min_position']; del entry['max_position']
        self.write('joint_limits.yaml', self.limits)
        self.assertEqual(self.load(urdf=str(path))['limits']['position_lower'], [-1., -1.])
        self.limits['joint_limits']['a']['max_velocity'] = 3.
        self.write('joint_limits.yaml', self.limits)
        with self.assertRaisesRegex(ValueError, 'exceed URDF'):
            self.load(urdf=str(path))

    def test_duplicate_yaml_ids_are_rejected(self):
        (self.root / 'config/advrf_hardware.yaml').write_text(
            'joint_map:\n  1: a\n  1: b\nprofiles: {}\n')
        with self.assertRaisesRegex(ValueError, 'Duplicate YAML key'):
            self.load()

    def test_reject_bad_mapping_and_profiles(self):
        for change in ('string_id', 'duplicate_name', 'missing_joint', 'missing_mode', 'bad_gains', 'nan'):
            with self.subTest(change=change):
                h = copy.deepcopy(self.hardware)
                if change == 'string_id': h['joint_map'] = {'1': 'a', 7: 'b'}
                if change == 'duplicate_name': h['joint_map'] = {1: 'a', 7: 'a'}
                if change == 'missing_joint': h['joint_map'] = {1: 'a'}
                if change == 'missing_mode': del h['profiles']['position']
                if change == 'bad_gains': h['profiles']['position']['a']['gains'] = [1.]
                if change == 'nan': h['profiles']['position']['a']['gains'][0] = float('nan')
                self.write('advrf_hardware.yaml', h)
                with self.assertRaises(ValueError): self.load()

    def test_reject_conflicting_modes_and_duplicate_limit_sources(self):
        self.write('advrf_controllers.yaml', {
            '/**/motor_power_node': {'ros__parameters': {'control_mode': 'impedance'}}})
        with self.assertRaisesRegex(ValueError, 'must match'): self.load()
        self.write('advrf_controllers.yaml', {
            '/**/joint_go_to_node': {'ros__parameters': {'max_velocity': [99., 99.]}}})
        with self.assertRaisesRegex(ValueError, 'shared joint limits'): self.load()


if __name__ == '__main__':
    unittest.main()
