"""Configuration and feedback validation, independent of Qt and ROS."""
import math
from pathlib import Path
import yaml


def moveit_parameters(document, path):
    """Merge MoveIt limits with the robot URDF; retain URDF joint order."""
    import xml.etree.ElementTree as ET
    config = Path(path).resolve().parent
    if config.name == 'controller':
        config = config.parent
    candidates = [p for p in (config.parent / 'urdf').glob('*.urdf')
                  if '.' not in p.stem]
    entries = document['joint_limits']
    urdf = {}
    if len(candidates) == 1:
        for joint in ET.parse(candidates[0]).getroot().findall('joint'):
            limit = joint.find('limit')
            if joint.get('type') != 'fixed' and limit is not None:
                urdf[joint.get('name')] = limit.attrib
    names = [name for name in urdf if name in entries]
    names += [name for name in entries if name not in names]
    result = {key: [] for key in ('joints', 'position_lower', 'position_upper',
                                   'max_velocity', 'max_acceleration')}
    for name in names:
        entry = entries[name]
        base = urdf.get(name, {})
        values = [entry.get('min_position', base.get('lower')),
                  entry.get('max_position', base.get('upper')),
                  entry.get('max_velocity', base.get('velocity')),
                  entry.get('max_acceleration')]
        try:
            values = [float(value) for value in values]
        except (TypeError, ValueError) as error:
            raise ValueError(f'{path}: {name}: missing position, velocity or acceleration limits; '
                             'provide min_position/max_position or one robot URDF in ros2/urdf') from error
        if (any(not math.isfinite(v) for v in values) or values[0] > values[1]
                or min(values[2:]) <= 0 or entry.get('has_velocity_limits') is False
                or entry.get('has_acceleration_limits') is False):
            raise ValueError(f'{path}: {name}: finite enabled joint limits are required')
        result['joints'].append(name)
        for key, value in zip(list(result)[1:], values):
            result[key].append(value)
    if not names:
        raise ValueError(f'{path}: empty joint_limits')
    return result


def load_parameters(path, namespace=''):
    with open(path) as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError('Expected ROS parameters YAML')
    if 'joint_limits' in document:
        return moveit_parameters(document, path)
    params = {}
    full_name = '/' + '/'.join(filter(None, [namespace.strip('/'), 'joint_goal_manager_node']))
    grouped_name = '/' + '/'.join(filter(None, [namespace.strip('/'), 'controllers', 'joint_goal_manager_node']))
    for key in ('/**', '/**/joint_goal_manager_node', 'joint_goal_manager_node', full_name, grouped_name):
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
    from .launch_config import ros2_folder, configuration_path
    if explicit:
        path = Path(explicit).expanduser()
    elif ros2_cfg:
        path = configuration_path(ros2_cfg, 'joint_limits')
    elif ec_cfg:
        ec_path = Path(ec_cfg).expanduser().resolve(strict=True)
        path = configuration_path(ros2_folder(ec_path) / 'config', 'joint_limits')
    else:
        raise ValueError('Choose joint_limits.yaml or set EC_CFG / ROS2_CFG')
    path = path.resolve(strict=True)
    if not path.is_file():
        raise ValueError(f'Expected a configuration file: {path}')
    return str(path)


def load_velocities(path, namespace, joints):
    values = load_parameters(path, namespace).get('max_velocity', [])
    if len(values) != len(joints) or any(not math.isfinite(v) or v <= 0 for v in values):
        raise ValueError('Expected positive max_velocity for every joint')
    return values
