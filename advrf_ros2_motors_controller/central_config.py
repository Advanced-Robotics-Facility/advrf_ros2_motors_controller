"""Single motor definition, scalar ROS parameters and consumer adapters."""
import re
from .robot_config import read_yaml, number, limits_from_entries, MODES

FLOAT_MAX = 3.402823466e38
GAIN_KEYS = tuple(f'gain_{i}' for i in range(5))


def flatten(document, prefix=''):
    result = {}
    for key, value in document.items():
        name = f'{prefix}.{key}' if prefix else str(key)
        if isinstance(value, dict):
            result.update(flatten(value, name))
        else:
            result[name] = value
    return result


def unflatten(parameters):
    result = {}
    for name, value in parameters.items():
        target = result
        keys = name.split('.')
        for key in keys[:-1]:
            target = target.setdefault(key, {})
        target[keys[-1]] = value
    return result


def load_central(path):
    document = read_yaml(path)
    try:
        return document['/**/robot_config_node']['ros__parameters']
    except KeyError as error:
        raise ValueError(f'{path}: expected /**/robot_config_node.ros__parameters') from error


def gain_range(profile, key):
    bounds = profile.get('gain_bounds', {}).get(key, {})
    if set(bounds) - {'min', 'max'}:
        raise ValueError(f'{key}: bounds support min and max only')
    lower = number(bounds.get('min', -FLOAT_MAX), f'{key} minimum')
    upper = number(bounds.get('max', FLOAT_MAX), f'{key} maximum')
    if not -FLOAT_MAX <= lower <= upper <= FLOAT_MAX:
        raise ValueError(f'{key}: invalid float32 gain bounds')
    return lower, upper


def adapt_central(document, joints=None, description=''):
    if set(document) - {'motors', 'planning', 'control_mode'}:
        raise ValueError('Robot configuration supports motors, planning and control_mode only')
    motors = document.get('motors')
    if not isinstance(motors, dict) or not motors:
        raise ValueError('Define at least one motor')
    joints = list(motors) if joints is None else joints
    if len(joints) != len(set(joints)) or any(j not in motors for j in joints):
        raise ValueError('Controller joints must be unique and present in robot_config.yaml')
    entries, mapping, profiles = {}, {}, {}
    for name, motor in motors.items():
        if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', name):
            raise ValueError(f'{name}: motor name must be usable as a ROS joint/parameter name')
        if set(motor) != {'id', 'limits', 'profiles'}:
            raise ValueError(f'{name}: expected id, limits and profiles')
        identifier = motor['id']
        if type(identifier) is not int or not 0 <= identifier <= 2147483647 or identifier in mapping:
            raise ValueError(f'{name}: ID must be a unique nonnegative int32')
        mapping[identifier] = name
        limits = motor['limits']
        if set(limits) != {'position_lower', 'position_upper', 'max_velocity', 'max_acceleration'}:
            raise ValueError(f'{name}: define all four operating limits')
        entries[name] = {
            'has_position_limits': True, 'min_position': limits['position_lower'],
            'max_position': limits['position_upper'], 'has_velocity_limits': True,
            'max_velocity': limits['max_velocity'], 'has_acceleration_limits': True,
            'max_acceleration': limits['max_acceleration'],
        }
        if not isinstance(motor['profiles'], dict) or not motor['profiles']:
            raise ValueError(f'{name}: define at least one gain profile')
        for mode, profile in motor['profiles'].items():
            if mode not in MODES or set(profile) - {'gain_bounds'} != set(GAIN_KEYS):
                raise ValueError(f'{name}/{mode}: expected gain_0 through gain_4')
            if set(profile.get('gain_bounds', {})) - set(GAIN_KEYS):
                raise ValueError(f'{name}/{mode}: unknown gain bound')
            values = []
            for key in GAIN_KEYS:
                value = number(profile[key], f'{name}/{mode}/{key}')
                lower, upper = gain_range(profile, key)
                if not lower <= value <= upper:
                    raise ValueError(f'{name}/{mode}/{key}: gain outside configured bounds')
                values.append(value)
            if mode in ('position', 'velocity') and any(v != 0 for v in values[3:]):
                raise ValueError(f'{name}/{mode}: native profile gain_3/gain_4 must be zero')
            profiles.setdefault(mode, {})[name] = {'gains': values}
    mode = document.get('control_mode', 'position')
    if mode not in MODES or any(mode not in motor['profiles'] for motor in motors.values()):
        raise ValueError(f'Every motor must define the selected control_mode {mode}')
    # Validate even motors outside this controller, preserving one valid definition.
    limits_from_entries(entries, list(motors), description)
    limits = limits_from_entries(entries, joints, description)
    planning = document.get('planning', {})
    if set(planning) - {'default_velocity_scaling_factor', 'default_acceleration_scaling_factor'}:
        raise ValueError('Unknown planning setting')
    planning = {key: number(planning.get(key, .1), key) for key in
                ('default_velocity_scaling_factor', 'default_acceleration_scaling_factor')}
    if any(not 0 < value <= 1 for value in planning.values()):
        raise ValueError('Planning scaling factors must be in (0, 1]')
    # Normalize numeric leaves to double except integer motor IDs.
    parameters = flatten(document)
    for key in parameters:
        if not key.endswith('.id') and key != 'control_mode':
            parameters[key] = number(parameters[key], key)
    return {'limits': limits, 'hardware': {'joint_map': mapping, 'profiles': profiles},
            'parameters': parameters, 'moveit_limits': {**planning, 'joint_limits': entries}}
