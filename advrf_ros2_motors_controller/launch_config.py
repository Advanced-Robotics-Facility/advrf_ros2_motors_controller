"""Resolve robot configuration paths from EC_CFG at launch time."""

import os
import math
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def robot_reference(ec_config, key):
    """Use the same PWD and relative path semantics as the middleware."""
    with ec_config.open() as stream:
        document = yaml.safe_load(stream)
    robot = document.get('robot') if isinstance(document, dict) else None
    reference = robot.get(key) if isinstance(robot, dict) else None
    if not isinstance(reference, str) or not reference.strip():
        raise ValueError(f'{ec_config}: robot.{key} is required')

    reference = reference.replace('${PWD}', str(ec_config.parent))
    reference = reference.replace('$PWD', str(ec_config.parent))
    path = Path(reference)
    return path if path.is_absolute() else ec_config.parent / path


def configuration_path(directory, config_name):
    """Find controller files in config/controller, with legacy config support."""
    directory = Path(directory).expanduser()
    if config_name in ('joint_limits', 'trajectory_config'):
        base = directory.parent if directory.name == 'controller' else directory
        direct = base / f'{config_name}.yaml'
        if direct.is_file():
            return direct
    nested = directory / 'controller' / f'{config_name}.yaml'
    return nested if nested.is_file() else directory / f'{config_name}.yaml'


def resolve_paths(config_file, robot_id_map, ec_cfg, ros2_cfg, config_name):
    """Explicit paths override ROS2_CFG, then references in EC_CFG."""
    needs_ec_config = not robot_id_map or (not config_file and not ros2_cfg)
    ec_config = (
        Path(ec_cfg).expanduser().resolve(strict=True)
        if ec_cfg and needs_ec_config else None
    )
    if not config_file:
        if ros2_cfg:
            directory = Path(ros2_cfg).expanduser()
        elif ec_config:
            directory = robot_reference(ec_config, 'robot_ros2_folder') / 'config'
        else:
            raise ValueError('Set EC_CFG or ROS2_CFG, or pass an explicit configuration file')
        config_file = configuration_path(directory, config_name)

    if not robot_id_map:
        if not ec_config:
            raise ValueError('Set EC_CFG or pass robot_id_map:=/path/to/robot_id_map.yaml')
        robot_id_map = robot_reference(ec_config, 'robot_id_map_path')

    paths = []
    for label, value in [('controller configuration', config_file), ('robot_id_map', robot_id_map)]:
        path = Path(value).expanduser().resolve(strict=True)
        if not path.is_file():
            raise ValueError(f'{label}: expected a file: {path}')
        paths.append(str(path))
    return tuple(paths)


def optional_config(explicit_path, joint_limits, basename=None):
    """An explicit override must exist; an absent default override is optional."""
    if explicit_path:
        path = Path(explicit_path).expanduser().resolve(strict=True)
        if not path.is_file():
            raise ValueError(f'Expected a configuration file: {path}')
        return str(path)
    if basename:
        path = configuration_path(Path(joint_limits).parent, basename)
        if path.exists():
            if not path.is_file():
                raise ValueError(f'Expected a configuration file: {path}')
            return str(path)
    return None


def controller_launch(config_name, executable, with_trajectory_controller=False, motor_power=False):
    uses_trajectory = config_name == 'trajectory_config'

    def launch_controller(context):
        joint_limits, id_map = resolve_paths(
            LaunchConfiguration('joint_limits').perform(context),
            LaunchConfiguration('robot_id_map').perform(context),
            os.environ.get('EC_CFG', ''),
            os.environ.get('ROS2_CFG', ''),
            'joint_limits',
        )
        from .panel_config import load_parameters
        with open(joint_limits) as stream:
            is_moveit = 'joint_limits' in (yaml.safe_load(stream) or {})
        shared_files = [load_parameters(joint_limits, LaunchConfiguration('namespace').perform(context))
                        if is_moveit else joint_limits]
        if uses_trajectory:
            trajectory_config = optional_config(
                LaunchConfiguration('trajectory_config').perform(context),
                joint_limits, 'trajectory_config',
            )
            if trajectory_config:
                shared_files.append(trajectory_config)

        from .motor_profiles import profile_parameters
        profile = profile_parameters(joint_limits,
            LaunchConfiguration('namespace').perform(context),
            LaunchConfiguration('control_mode').perform(context),
            LaunchConfiguration('motor_profiles_dir').perform(context),
            LaunchConfiguration('motor_profile').perform(context),
            os.environ.get('EC_CFG', ''))
        shared_files.append({'command_enabled': ParameterValue(
            LaunchConfiguration('command_enabled'), value_type=bool)})
        if profile:
            shared_files.append({'joint_gains': profile['joint_gains']})
            from launch.logging import get_logger
            get_logger('motor_profiles').info(f"Motor profile: {profile['path']} (joints from {joint_limits})")

        node_config = optional_config(
            LaunchConfiguration('node_config').perform(context), joint_limits,
        )
        parameters = shared_files + ([node_config] if node_config else [])
        parameters.append({'robot_id_map': id_map})
        if motor_power:
            parameters.append({
                'control_mode': LaunchConfiguration('control_mode'),
                'auto_start_motors': ParameterValue(
                    LaunchConfiguration('auto_start_motors'), value_type=bool,
                ),
            })

            if profile:
                parameters[-1]['start_joint_gains'] = profile['start_joint_gains']
            explicit_gains = LaunchConfiguration('start_gains').perform(context)
            if explicit_gains:
                gains = yaml.safe_load(explicit_gains)
                if (not isinstance(gains, list) or len(gains) != 5
                        or any(isinstance(v, bool) or not isinstance(v, (int, float))
                               or not math.isfinite(v) for v in gains)):
                    raise ValueError('start_gains must be a YAML list of five finite numbers')
                parameters[-1]['start_gains'] = [float(v) for v in gains]

        nodes = [Node(
            package='advrf_ros2_motors_controller',
            executable=executable,
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=parameters,
        )]
        if with_trajectory_controller:
            nodes.append(Node(
                package='advrf_ros2_motors_controller',
                executable='joint_trajectory_controller_node',
                namespace=LaunchConfiguration('namespace'),
                output='screen',
                parameters=shared_files + [{'robot_id_map': id_map}],
                condition=IfCondition(LaunchConfiguration('start_trajectory_controller')),
            ))
        return nodes

    arguments = [
        DeclareLaunchArgument(
            'joint_limits', default_value='',
            description='Shared joint parameters; defaults to ROS2_CFG or EC_CFG robot_ros2_folder/config/controller',
        ),
        DeclareLaunchArgument(
            'node_config', default_value='',
            description='Optional parameters YAML overriding only the requested node',
        ),
        DeclareLaunchArgument(
            'robot_id_map', default_value='',
            description='Joint map YAML; defaults to EC_CFG robot.robot_id_map_path',
        ),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('command_enabled', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('motor_profile', default_value='', description='Explicit middleware motor profile YAML'),
        DeclareLaunchArgument('motor_profiles_dir', default_value='', description='Profiles directory; defaults beside EC_CFG robot_control_path'),
        DeclareLaunchArgument('control_mode', default_value='position',
            choices=['position', 'impedance', 'velocity', 'torque', 'current']),
    ]
    if uses_trajectory:
        arguments.append(DeclareLaunchArgument(
            'trajectory_config', default_value='',
            description='Optional trajectory overrides; defaults beside joint_limits.yaml',
        ))
    if motor_power:
        arguments.extend([
            DeclareLaunchArgument('start_gains', default_value='',
                description='Optional explicit START gains [pos_kp, pos_kd, tor_kp, tor_ki, tor_kd]'),
            DeclareLaunchArgument('auto_start_motors', default_value='false',
                                  choices=['true', 'false']),
        ])
    if with_trajectory_controller:
        arguments.append(DeclareLaunchArgument(
            'start_trajectory_controller', default_value='true',
            description='Start the trajectory controller alongside GoTo',
        ))
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_controller)])
