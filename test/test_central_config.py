import copy
import unittest
from advrf_ros2_motors_controller.central_config import adapt_central, flatten, unflatten


class CentralConfigTest(unittest.TestCase):
    def setUp(self):
        self.document = {'control_mode': 'position', 'planning': {'default_velocity_scaling_factor': .2},
            'motors': {'a': {'id': 7, 'limits': {'position_lower': -1., 'position_upper': 1.,
            'max_velocity': 2., 'max_acceleration': 3.}, 'profiles': {'position': {
            'gain_0': .009, 'gain_1': 0., 'gain_2': .1, 'gain_3': 0., 'gain_4': 0.,
            'gain_bounds': {'gain_0': {'min': 0., 'max': .02}}}}}}}

    def test_scalars_and_moveit_adapter_share_definition(self):
        result = adapt_central(self.document)
        self.assertEqual(result['parameters']['motors.a.profiles.position.gain_0'], .009)
        self.assertEqual(result['hardware']['profiles']['position']['a']['gains'], [.009, 0., .1, 0., 0.])
        self.assertEqual(result['limits']['position_lower'], [-1.])
        entry = result['moveit_limits']['joint_limits']['a']
        self.assertEqual(entry['max_velocity'], result['limits']['max_velocity'][0])
        self.assertEqual(result['moveit_limits']['default_velocity_scaling_factor'], .2)
        self.assertEqual(unflatten(flatten(self.document)), self.document)

    def test_bounds_and_gain_shape_validation(self):
        for error in ('bound', 'nan', 'missing_gain', 'bad_slot', 'mode'):
            d = copy.deepcopy(self.document)
            gains = d['motors']['a']['profiles']['position']
            if error == 'bound': gains['gain_0'] = .03
            if error == 'nan': gains['gain_0'] = float('nan')
            if error == 'missing_gain': del gains['gain_1']
            if error == 'bad_slot': gains['gain_4'] = 1.
            if error == 'mode': d['control_mode'] = 'velocity'
            with self.subTest(error=error), self.assertRaises(ValueError): adapt_central(d)

    def test_limits_cannot_exceed_urdf(self):
        model = '<robot><joint name="a" type="revolute"><limit lower="-1" upper="1" velocity="1"/></joint></robot>'
        with self.assertRaisesRegex(ValueError, 'exceed URDF'):
            adapt_central(self.document, description=model)


if __name__ == '__main__':
    unittest.main()
