"""Configuration and feedback validation, independent of Qt and ROS."""
import math
from pathlib import Path
import yaml


def load_parameters(path, namespace=''):
    with open(path) as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError('Expected ROS parameters YAML')
    params = {}
    full_name = '/' + '/'.join(filter(None, [namespace.strip('/'), 'joint_go_to_node']))
    for key in ('/**', '/**/joint_go_to_node', 'joint_go_to_node', full_name):
        block = document.get(key, {})
        params.update(block.get('ros__parameters', {}))
    return params


def load_joints(path, namespace=''):
    params = load_parameters(path, namespace)
    names = params.get('joints', [])
    lower, upper = params.get('position_lower', []), params.get('position_upper', [])
    if (not names or any(not isinstance(n, str) or not n for n in names)
            or len(set(names)) != len(names)
            or len(lower) != len(names) or len(upper) != len(names)):
        raise ValueError('Expected unique joints and position_lower/upper for every joint')
    for lo, hi in zip(lower, upper):
        if not math.isfinite(lo) or not math.isfinite(hi) or lo > hi:
            raise ValueError('Invalid position limits')
    return list(zip(names, lower, upper))


def measured_positions(names, positions, joints):
    if len(names) != len(positions) or len(set(names)) != len(names):
        raise ValueError('Malformed JointState')
    values = dict(zip(names, positions))
    result = [values[name] for name, _, _ in joints]
    if not all(math.isfinite(value) for value in result):
        raise ValueError('Nonfinite feedback')
    return result


def resolve_panel_config(explicit='', ec_cfg='', ros2_cfg=''):
    """A panel needs only joint configuration, not the hardware ID map."""
    from .launch_config import robot_reference
    if explicit:
        path = Path(explicit).expanduser()
    elif ros2_cfg:
        path = Path(ros2_cfg).expanduser() / 'joint_config.yaml'
    elif ec_cfg:
        ec_path = Path(ec_cfg).expanduser().resolve(strict=True)
        path = robot_reference(ec_path, 'robot_ros2_folder') / 'config/joint_config.yaml'
    else:
        raise ValueError('Choose joint_config.yaml or set EC_CFG / ROS2_CFG')
    path = path.resolve(strict=True)
    if not path.is_file():
        raise ValueError(f'Expected a configuration file: {path}')
    return str(path)


def load_velocities(path, namespace, joints):
    values = load_parameters(path, namespace).get('max_velocity', [])
    if len(values) != len(joints) or any(not math.isfinite(v) or v <= 0 for v in values):
        raise ValueError('Expected positive max_velocity for every joint')
    return values
