"""Reject missing and inconsistent controller parameters before motion."""
import unittest

from rclpy.parameter import Parameter
from advrf_ros2_motors_controller.panel_config import (
    TRAJECTORY_PARAMETER_NAMES,
    decode_joint_parameters,
)


class ParameterLimitsTest(unittest.TestCase):
    def values(self, **overrides):
        parameters = {
            'joints': ['a', 'b'],
            'position_lower': [-1.0, -1.0],
            'position_upper': [1.0, 1.0],
            'max_velocity': [2.0, 2.0],
            'max_acceleration': [3.0, 3.0],
        }
        parameters.update(overrides)
        return [Parameter(name, value=parameters[name]).get_parameter_value()
                for name in TRAJECTORY_PARAMETER_NAMES]

    def test_complete_parameters(self):
        result = decode_joint_parameters(self.values(), TRAJECTORY_PARAMETER_NAMES)
        self.assertEqual(result['joints'], ['a', 'b'])
        self.assertEqual(result['max_acceleration'], [3.0, 3.0])

    def test_reject_invalid_or_missing_parameters(self):
        for changes in (
            {'joints': ['a', 'a']},
            {'position_upper': [-2.0, 1.0]},
            {'max_velocity': [0.0, 2.0]},
            {'max_acceleration': [float('nan'), 3.0]},
            {'position_lower': [-1.0]},
        ):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                decode_joint_parameters(self.values(**changes), TRAJECTORY_PARAMETER_NAMES)

        with self.assertRaises(ValueError):
            decode_joint_parameters(self.values()[:-1], TRAJECTORY_PARAMETER_NAMES)


if __name__ == '__main__':
    unittest.main()
