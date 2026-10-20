"""Check configuration precedence without starting ROS nodes."""

from pathlib import Path
import tempfile
import unittest
import yaml
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.utilities import evaluate_parameters
from advrf_ros2_motors_controller.launch_config import controller_launch

from advrf_ros2_motors_controller.launch_config import resolve_paths


class LaunchConfigTest(unittest.TestCase):
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
                for name in ('trajectory_config', 'joint_config'):
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
                resolve_paths('', '', str(ec_cfg), '/missing/config', 'joint_config')
            with self.assertRaisesRegex(ValueError, 'Set EC_CFG'):
                resolve_paths('', '', '', '', 'joint_config')
            ec_cfg.write_text('robot: {}\n')
            with self.assertRaisesRegex(ValueError, 'robot.robot_ros2_folder'):
                resolve_paths('', '', str(ec_cfg), '', 'joint_config')

    def test_go_to_launch_shares_config_and_optional_controller(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            config = root / 'trajectory_config.yaml'
            config.write_text('/**: {ros__parameters: {feedback_timeout: 0.4}}\n')
            joint = root / 'joint_config.yaml'
            joint.write_text('/**: {ros__parameters: {joints: [a], max_velocity: [1.0]}}\n')
            mapping = root / 'robot_id_map.yaml'
            mapping.write_text('joint_map: {1: a}\n')
            with patch.dict('os.environ', {'ROS_LOG_DIR': str(root)}):
                for start in ('true', 'false'):
                    context = LaunchContext()
                    context.launch_configurations.update({
                        'joint_config': str(joint),
                        'trajectory_config': str(config),
                        'robot_id_map': str(mapping),
                        'start_trajectory_controller': start,
                    })
                    description = controller_launch(
                        'trajectory_config', 'joint_go_to_node', with_trajectory_controller=True,
                    )
                    for entity in description.entities:
                        if isinstance(entity, DeclareLaunchArgument):
                            entity.execute(context)
                    callback = next(entity for entity in description.entities
                                    if isinstance(entity, OpaqueFunction))
                    nodes = callback.execute(context)
                    self.assertEqual(len(nodes), 2)
                    self.assertEqual(nodes[1].condition.evaluate(context), start == 'true')
                    for node in nodes:
                        params = evaluate_parameters(context, node._Node__parameters)
                        self.assertEqual(str(params[0]), str(joint))
                        self.assertEqual(str(params[1]), str(config))
                        self.assertEqual(params[-1]['robot_id_map'], str(mapping))

    def test_motor_power_launch_requires_explicit_auto_start(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            config = root / 'trajectory_config.yaml'
            config.write_text('/**: {ros__parameters: {feedback_timeout: 0.4}}\n')
            joint = root / 'joint_config.yaml'
            joint.write_text('/**: {ros__parameters: {joints: [a], max_velocity: [1.0]}}\n')
            mapping = root / 'robot_id_map.yaml'
            mapping.write_text('joint_map: {1: a}\n')
            with patch.dict('os.environ', {'ROS_LOG_DIR': str(root)}):
                for auto_start in (False, True):
                    context = LaunchContext()
                    context.launch_configurations.update({
                        'joint_config': str(joint),
                        'robot_id_map': str(mapping),
                        'control_mode': 'impedance',
                        'start_gains': '[0.1, 0.0, 0.0005, 0.0, 0.0]',
                    })
                    if auto_start:
                        context.launch_configurations['auto_start_motors'] = 'true'
                    description = controller_launch(
                        'joint_config', 'motor_power_node', motor_power=True,
                    )
                    for entity in description.entities:
                        if isinstance(entity, DeclareLaunchArgument):
                            entity.execute(context)
                    callback = next(entity for entity in description.entities
                                    if isinstance(entity, OpaqueFunction))
                    nodes = callback.execute(context)
                    self.assertEqual(len(nodes), 1)
                    params = evaluate_parameters(context, nodes[0]._Node__parameters)
                    self.assertEqual(params[-1]['control_mode'], 'impedance')
                    self.assertEqual(list(params[-1]['start_gains']), [0.1, 0.0, 0.0005, 0.0, 0.0])
                    self.assertIs(params[-1]['auto_start_motors'], auto_start)

    def test_optional_overrides_and_precedence(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            joint = root / 'joint_config.yaml'
            joint.write_text('/**: {ros__parameters: {joints: [a], feedback_timeout: 0.2}}\n')
            trajectory = root / 'trajectory_config.yaml'
            mapping = root / 'robot_id_map.yaml'
            mapping.write_text('joint_map: {1: a}\n')
            override = root / 'go_to_config.yaml'
            override.write_text('/**: {ros__parameters: {feedback_timeout: 0.8}}\n')

            def nodes(extra=None):
                context = LaunchContext()
                context.launch_configurations.update({
                    'joint_config': str(joint), 'robot_id_map': str(mapping),
                })
                context.launch_configurations.update(extra or {})
                description = controller_launch(
                    'trajectory_config', 'joint_go_to_node', with_trajectory_controller=True,
                )
                for entity in description.entities:
                    if isinstance(entity, DeclareLaunchArgument):
                        entity.execute(context)
                callback = next(entity for entity in description.entities
                                if isinstance(entity, OpaqueFunction))
                return context, callback.execute(context)

            def effective(node, context):
                parameters = {}
                for entry in evaluate_parameters(context, node._Node__parameters):
                    if isinstance(entry, dict):
                        parameters.update(entry)
                    else:
                        parameters.update(yaml.safe_load(entry.read_text())['/**']['ros__parameters'])
                return parameters

            with patch.dict('os.environ', {'ROS_LOG_DIR': str(root)}):
                # Missing automatic trajectory overrides use the common file and node defaults.
                context, actions = nodes()
                self.assertEqual(effective(actions[0], context)['feedback_timeout'], 0.2)
                trajectory.write_text('/**: {ros__parameters: {feedback_timeout: 0.4}}\n')
                context, actions = nodes({'node_config': str(override)})
                self.assertEqual(effective(actions[0], context)['feedback_timeout'], 0.8)
                self.assertEqual(effective(actions[1], context)['feedback_timeout'], 0.4)
                for action in actions:
                    self.assertEqual(effective(action, context)['joints'], ['a'])
                with self.assertRaises(FileNotFoundError):
                    nodes({'trajectory_config': str(root / 'missing.yaml')})
                with self.assertRaises(FileNotFoundError):
                    nodes({'node_config': str(root / 'missing.yaml')})


if __name__ == '__main__':
    unittest.main()
