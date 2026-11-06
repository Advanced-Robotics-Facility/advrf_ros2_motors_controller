"""Resolve robot configuration paths from EC_CFG at launch time."""

import os
from pathlib import Path

import yaml
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.substitution import Substitution


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
    path = Path(os.path.expandvars(reference))
    return path if path.is_absolute() else ec_config.parent / path


def ros2_folder(ec_config):
    """Prefer ros2.folder, while accepting the former robot key."""
    with Path(ec_config).open() as stream:
        document = yaml.safe_load(stream)
    section = document.get('ros2', {}) if isinstance(document, dict) else {}
    if 'folder' not in section:
        return robot_reference(Path(ec_config), 'robot_ros2_folder')
    reference = section['folder']
    if not isinstance(reference, str) or not reference.strip():
        raise ValueError(f'{ec_config}: ros2.folder must be a nonempty path')
    base = Path(ec_config).expanduser().resolve().parent
    path = Path(os.path.expandvars(reference.replace('${PWD}', str(base)).replace('$PWD', str(base)))).expanduser()
    return path if path.is_absolute() else base / path


def resolve_motor_profile(explicit='', directory='', ec_cfg='', motor_power=False):
    """Use a DDS TX profile only for command conversion, never for motor START."""
    if explicit or directory or motor_power or not ec_cfg:
        return explicit
    config = Path(ec_cfg).expanduser().resolve(strict=True)
    with config.open() as stream:
        document = yaml.safe_load(stream) or {}
    reference = document.get('ros2', {}).get('motor_tx_profile', '')
    if not reference:
        return ''
    if not isinstance(reference, str):
        raise ValueError('ros2.motor_tx_profile must be a path string')
    value = reference.replace('${PWD}', str(config.parent)).replace('$PWD', str(config.parent))
    path = Path(os.path.expandvars(value)).expanduser()
    if not path.is_absolute():
        path = config.parent / path
    return str(path.resolve(strict=True))


def resolve_namespace(explicit=None, ec_cfg=''):
    """An explicit namespace (including /) overrides the robot default."""
    if explicit is not None:
        return explicit
    if not ec_cfg:
        return ''
    with Path(ec_cfg).expanduser().open() as stream:
        document = yaml.safe_load(stream)
    section = document.get('ros2', {}) if isinstance(document, dict) else {}
    namespace = section.get('namespace', '')
    if not isinstance(namespace, str):
        raise ValueError(f'{ec_cfg}: ros2.namespace must be a string')
    namespace = namespace.strip()
    if not namespace:
        return ''
    from rclpy.validate_namespace import validate_namespace
    namespace = '/' + namespace.strip('/')
    validate_namespace(namespace)
    return namespace


def resolve_ecat_namespace(explicit=None, ec_cfg='', namespace=''):
    """Keep low-level transport topics separate from high-level ROS interfaces."""
    value = explicit
    if not value and ec_cfg:
        with Path(ec_cfg).expanduser().open() as stream:
            document = yaml.safe_load(stream) or {}
        native = document.get('advrf_ros2_ecat', document.get('ec_ros2'))
        section = native if native is not None else document.get('ecat_dds')
        if section is None:
            section = document.get('dds_bridge', {})
        if not isinstance(section, dict):
            raise ValueError('ecat_dds must be a mapping')
        value = section.get('namespace')
        if not value and native is not None:
            base = (namespace or resolve_namespace(ec_cfg=ec_cfg) or '/').rstrip('/')
            value = base + '/ecat'
    if not value:
        # Preserve existing middleware configurations without a separate DDS prefix.
        value = namespace or resolve_namespace(ec_cfg=ec_cfg) or '/'
    if not isinstance(value, str):
        raise ValueError('ecat_namespace must be a string')
    value = '/' + value.strip().strip('/')
    from rclpy.validate_namespace import validate_namespace
    validate_namespace(value)
    return value


def ecat_remappings(namespace):
    prefix = namespace.rstrip('/')
    return [(name, prefix + '/' + name)
            for name in ('rx/joint_states', 'rx/motors', 'tx/motors', 'replCmd')]


def grouped_namespace(namespace, group):
    return '/' + '/'.join(filter(None, (namespace.strip('/'), group)))


def robot_remappings(namespace):
    """Resolve public interfaces under their functional namespace."""
    prefix = namespace.rstrip('/')
    return [('joint_commands', grouped_namespace(namespace, 'controllers') + '/joint_commands'),
            ('selected_joint', grouped_namespace(namespace, 'inputs') + '/selected_joint')] + [
        (name, prefix + '/' + name) for name in (
        'start_motors', 'stop_motors',
        'motor_power_request', 'set_trajectory_enabled', 'go_to_joint_positions',
        'advrf_motors_controller/follow_joint_trajectory')]


def config_argument(context, name):
    return LaunchConfiguration(name, default=EnvironmentVariable(name.upper(), default_value='')).perform(context)


def config_arguments():
    return [DeclareLaunchArgument(name, default_value=EnvironmentVariable(name.upper(), default_value=''))
            for name in ('ec_cfg', 'ros2_cfg')]


class RobotNamespace(Substitution):
    """Resolve the default at launch execution, after ec_cfg overrides."""
    def __init__(self, fallback=''):
        super().__init__()
        self.fallback = fallback

    def perform(self, context):
        return resolve_namespace(ec_cfg=config_argument(context, 'ec_cfg')) or self.fallback

    def describe(self):
        return 'namespace from EC_CFG ros2.namespace'


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
            directory = ros2_folder(ec_config) / 'config'
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
