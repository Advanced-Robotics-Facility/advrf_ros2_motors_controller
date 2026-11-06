"""Check configuration precedence without starting ROS nodes."""

from pathlib import Path
import tempfile
import unittest
import yaml
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

from advrf_ros2_motors_controller.launch_config import resolve_paths


class LaunchConfigTest(unittest.TestCase):
    def test_ros2_section_and_launch_overrides(self):
        from advrf_ros2_motors_controller.launch_config import RobotNamespace, config_arguments
        from advrf_ros2_motors_controller.panel_config import resolve_panel_config
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            config = root / 'ros2/config'
            config.mkdir(parents=True)
            limits = config / 'joint_limits.yaml'
            limits.write_text('/**: {ros__parameters: {joints: [a]}}')
            mapping = root / 'map.yaml'
            mapping.write_text('joint_map: {1: a}')
            ec = root / 'ec.yaml'
            for reference in ('${PWD}/ros2', '$PWD/ros2', 'ros2', str(root / 'ros2')):
                ec.write_text('robot: {robot_id_map_path: "${PWD}/map.yaml"}\n'
                              f'ros2: {{folder: "{reference}", namespace: /advrf/bench}}\n')
                self.assertEqual(resolve_paths('', '', str(ec), '', 'joint_limits'),
                                 (str(limits), str(mapping)))
                self.assertEqual(resolve_panel_config('', str(ec)), str(limits))
            with patch.dict('os.environ', {'EC_CFG': '/missing/env.yaml'}):
                context = LaunchContext()
                context.launch_configurations['ec_cfg'] = str(ec)
                for action in config_arguments():
                    action.execute(context)
                namespace = DeclareLaunchArgument('namespace', default_value=RobotNamespace())
                namespace.execute(context)
                self.assertEqual(context.launch_configurations['namespace'], '/advrf/bench')
                for override in ('/custom', '/'):
                    context.launch_configurations['namespace'] = override
                    namespace.execute(context)
                    self.assertEqual(context.launch_configurations['namespace'], override)
            # A full explicit invocation can ignore an unavailable EC_CFG.
            self.assertEqual(resolve_paths(str(limits), str(mapping), '/missing', '', 'joint_limits'),
                             (str(limits), str(mapping)))

    def test_controller_subdirectory(self):
        from advrf_ros2_motors_controller.panel_config import resolve_panel_config
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            config = root / 'ros2/config'
            directory = config / 'controller'
            directory.mkdir(parents=True)
            joint = directory / 'joint_limits.yaml'
            joint.write_text('/**: {ros__parameters: {joints: [a]}}')
            mapping = root / 'map.yaml'
            mapping.write_text('joint_map: {1: a}')
            ec = root / 'ec.yaml'
            ec.write_text('robot: {robot_ros2_folder: "${PWD}/ros2", robot_id_map_path: "${PWD}/map.yaml"}')
            for ros2_cfg in ('', str(config), str(directory)):
                paths = resolve_paths('', '', str(ec), ros2_cfg, 'joint_limits')
                self.assertEqual(paths, (str(joint), str(mapping)))
                self.assertEqual(resolve_panel_config('', str(ec), ros2_cfg), str(joint))

    def test_moveit_limits_and_relocated_trajectory(self):
        from advrf_ros2_motors_controller.panel_config import load_parameters
        from advrf_ros2_motors_controller.launch_config import optional_config
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            config = root / 'config'
            (config / 'controller').mkdir(parents=True)
            (root / 'urdf').mkdir()
            (root / 'urdf/robot.urdf').write_text(
                '<robot name="test"><joint name="a" type="revolute">'
                '<limit lower="-2" upper="3" velocity="4"/></joint></robot>')
            limits = config / 'joint_limits.yaml'
            limits.write_text('joint_limits: {a: {max_velocity: 2.0, max_acceleration: 1.0}}')
            params = load_parameters(limits)
            self.assertEqual(params['position_lower'], [-2.0])
            self.assertEqual(params['position_upper'], [3.0])
            self.assertEqual(params['max_velocity'], [2.0])
            trajectory = config / 'trajectory_config.yaml'
            trajectory.touch()
            self.assertEqual(optional_config('', limits, 'trajectory_config'), str(trajectory))
            self.assertEqual(optional_config('', config / 'controller/unused.yaml',
                                             'trajectory_config'), str(trajectory))
            limits.write_text('joint_limits: {unknown: {max_velocity: 1, max_acceleration: 1}}')
            with self.assertRaisesRegex(ValueError, 'missing position'):
                load_parameters(limits)


    def test_references_and_overrides(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            config = root / 'ros2/config'
            config.mkdir(parents=True)
            mapping = root / 'robot_id_map.yaml'
            mapping.write_text('joint_map: {1: a}\n')
            ec_cfg = root / 'ec_cfg.yaml'
            override = root / 'override'
            override.mkdir()

            for reference in ('${PWD}/ros2', '$PWD/ros2', 'ros2', str(root / 'ros2')):
                ec_cfg.write_text(
                    'robot:\n'
                    f'  robot_ros2_folder: "{reference}"\n'
                    '  robot_id_map_path: "${PWD}/robot_id_map.yaml"\n'
                )
                for name in ('trajectory_config', 'joint_limits'):
                    with self.subTest(reference=reference, name=name):
                        default_file = config / f'{name}.yaml'
                        default_file.touch()
                        override_file = override / f'{name}.yaml'
                        override_file.touch()
                        self.assertEqual(
                            resolve_paths('', '', str(ec_cfg), '', name),
                            (str(default_file), str(mapping)),
                        )
                        self.assertEqual(
                            resolve_paths('', '', str(ec_cfg), str(override), name),
                            (str(override_file), str(mapping)),
                        )
                        self.assertEqual(
                            resolve_paths(str(default_file), str(mapping), '/missing/ec.yaml',
                                          '/missing/override', name),
                            (str(default_file), str(mapping)),
                        )

            with self.assertRaises(FileNotFoundError):
                resolve_paths('', '', str(ec_cfg), '/missing/config', 'joint_limits')
            with self.assertRaisesRegex(ValueError, 'Set EC_CFG'):
                resolve_paths('', '', '', '', 'joint_limits')
            ec_cfg.write_text('robot: {}\n')
            with self.assertRaisesRegex(ValueError, 'robot.robot_ros2_folder'):
                resolve_paths('', '', str(ec_cfg), '', 'joint_limits')


if __name__ == '__main__':
    unittest.main()
