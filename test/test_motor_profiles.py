"""Mode selection, profile mapping and fail-closed configuration checks."""
from pathlib import Path
import tempfile
import unittest
from advrf_ros2_motors_controller.motor_profiles import profile_parameters


class MotorProfilesTest(unittest.TestCase):
    def test_native_wire_gains_preserve_start_gains(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / 'motor').mkdir()
            (root / 'motor/position.yaml').write_text('joint: {control_mode: 59, gains: [10, 2, 3, 0, 0]}')
            (root / 'joint.yaml').write_text('/**: {ros__parameters: {joints: [joint]}}')
            (root / 'ec.yaml').write_text('robot: {robot_control_path: "${PWD}/control.yaml"}\nadvrf_ros2_ecat: {}')
            params = profile_parameters(str(root / 'joint.yaml'), '', 'position', ec_cfg=str(root / 'ec.yaml'))
            self.assertEqual(params['joint_gains'], [10., 3., 0., 0., 2.])
            self.assertEqual(params['start_joint_gains'], [10., 2., 3., 0., 0.])

    def test_profiles(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            directory = root / 'control/motor'
            directory.mkdir(parents=True)
            joint = root / 'joint.yaml'
            joint.write_text('/**: {ros__parameters: {joints: [a, b]}}')
            ec = root / 'ec.yaml'
            ec.write_text('robot: {robot_control_path: "${PWD}/control/robot.yaml"}')
            position = directory / 'position.yaml'
            position.write_text('yaw: {control_mode: 0x3B, gains: [200, 0, 10, 0, 0]}\n'
                                'pitch: {control_mode: 0x3B, gains: [200, 0, 10, 0, 0]}')
            (directory / 'impedance.yaml').write_text(
                'yaw: {control_mode: 0xD4, gains: [500, 10, 1, 0.007, 0.7]}')
            result = profile_parameters(str(joint), '', 'position', ec_cfg=str(ec))
            self.assertEqual(result['path'], str(position))
            self.assertEqual(result['start_joint_gains'], [200., 0., 10., 0., 0.] * 2)
            self.assertEqual(result['joint_gains'], result['start_joint_gains'])
            impedance = profile_parameters(str(joint), '', 'impedance', directory=str(directory))
            self.assertEqual(impedance['joint_gains'], [500., 10., 1., 0.007, 0.7] * 2)
            # Different gains cannot be assigned by guessing joint order.
            position.write_text('yaw: {control_mode: 59, gains: [1, 2, 3, 4, 5]}\n'
                                'pitch: {control_mode: 59, gains: [6, 7, 8, 9, 10]}')
            with self.assertRaisesRegex(ValueError, 'profile_joints'):
                profile_parameters(str(joint), '', 'position', profile=str(position))
            joint.write_text('/**: {ros__parameters: {joints: [a, b], profile_joints: [pitch, yaw]}}')
            result = profile_parameters(str(joint), '', 'position', profile=str(position))
            self.assertEqual(result['joint_gains'], [6., 7., 8., 9., 10., 1., 2., 3., 4., 5.])
            with self.assertRaises(ValueError):
                profile_parameters(str(joint), '', 'velocity', directory=str(directory))
            (directory / 'duplicate.yaml').write_text(position.read_text())
            with self.assertRaisesRegex(ValueError, 'found 2'):
                profile_parameters(str(joint), '', 'position', directory=str(directory))
            position.write_text('yaw: {control_mode: 59, gains: [1, .nan, 3, 4, 5]}')
            with self.assertRaises(ValueError):
                profile_parameters(str(joint), '', 'position', profile=str(position))
            self.assertIsNone(profile_parameters(str(joint), '', 'position'))


if __name__ == '__main__':
    unittest.main()
