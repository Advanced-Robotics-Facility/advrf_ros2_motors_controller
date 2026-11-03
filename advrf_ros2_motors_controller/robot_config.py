"""Load ROS-only robot configuration; never discover data through EC_CFG."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

MODES = {'position', 'velocity', 'impedance', 'torque', 'current'}


class UniqueLoader(yaml.SafeLoader):
    """Reject accidental duplicate IDs/settings instead of silently keeping the last."""

    def construct_mapping(self, node, deep=False):
        self.flatten_mapping(node)
        result = {}
        for key_node, value_node in node.value:
            key = self.construct_object(key_node, deep=deep)
            if key in result:
                raise ValueError(f'Duplicate YAML key: {key}')
            result[key] = self.construct_object(value_node, deep=deep)
        return result


def read_yaml(path):
    with Path(path).open() as stream:
        value = yaml.load(stream, Loader=UniqueLoader)
    if not isinstance(value, dict):
        raise ValueError(f'{path}: expected a mapping')
    return value


def node_parameters(path, node, namespace=''):
    document = read_yaml(path)
    result = {}
    full = '/' + '/'.join(filter(None, [namespace.strip('/'), node]))
    for key in ('/**', f'/**/{node}', node, full):
        result.update(document.get(key, {}).get('ros__parameters', {}))
    return result


def robot_description(path):
    if not path:
        return ''
    path = Path(path).expanduser().resolve(strict=True)
    if path.suffix == '.xacro':
        import xacro
        return xacro.process_file(str(path)).toxml()
    return path.read_text()


def number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{label}: expected a finite number')
    return float(value)


def joint_limits(path, joints, description=''):
    return limits_from_entries(read_yaml(path)['joint_limits'], joints, description)


def limits_from_entries(entries, joints, description=''):
    physical = {}
    if description:
        for joint in ET.fromstring(description).findall('joint'):
            limit = joint.find('limit')
            if joint.get('type') != 'fixed' and limit is not None:
                physical[joint.get('name')] = {k: float(v) for k, v in limit.attrib.items()}
    result = {'joints': joints}
    for key in ('position_lower', 'position_upper', 'max_velocity', 'max_acceleration'):
        result[key] = []
    for name in joints:
        if description and name not in physical:
            raise ValueError(f'{name}: controlled joint missing from URDF limits')
        entry = entries.get(name)
        if not isinstance(entry, dict):
            raise ValueError(f'{name}: missing MoveIt limits')
        base = physical.get(name, {})
        values = [entry.get('min_position', base.get('lower')),
                  entry.get('max_position', base.get('upper')),
                  entry.get('max_velocity', base.get('velocity')),
                  entry.get('max_acceleration')]
        lo, hi, vel, acc = [number(v, f'{name} limits') for v in values]
        if any(entry.get(flag) is False for flag in
               ('has_position_limits', 'has_velocity_limits', 'has_acceleration_limits')):
            raise ValueError(f'{name}: finite enabled limits are required by ADVRF')
        if (lo > hi or vel <= 0 or acc <= 0 or
                lo < base.get('lower', lo) or hi > base.get('upper', hi) or
                vel > base.get('velocity', vel)):
            raise ValueError(f'{name}: invalid limits or limits exceed URDF')
        for key, value in zip(list(result)[1:], (lo, hi, vel, acc)):
            result[key].append(value)
    return result


def load_robot(config_dir, namespace, urdf='', control_mode=''):
    root = Path(config_dir).expanduser().resolve(strict=True)
    config = root / 'config'
    if (config / 'controller.yaml').is_file():
        document = {}
        controller = read_yaml(config / 'controller.yaml')['controller']
        name = controller['name']
    else:
        document = read_yaml(config / 'moveit_controllers.yaml')
        manager = document['moveit_simple_controller_manager']
        names = manager['controller_names']
        if len(names) != 1:
            raise ValueError('ADVRF currently requires one shared FollowJointTrajectory controller')
        name = names[0]
        controller = manager[name]
    joints = controller['joints']
    if (controller.get('type', 'FollowJointTrajectory') != 'FollowJointTrajectory' or not isinstance(joints, list)
            or not joints or any(not isinstance(j, str) or not j for j in joints)
            or len(set(joints)) != len(joints)):
        raise ValueError('Expected FollowJointTrajectory with unique nonempty joint names')
    action = controller.get('action_ns', 'follow_joint_trajectory')
    if not isinstance(action, str) or not action.strip('/'):
        raise ValueError('Invalid trajectory action namespace')
    prefix = namespace.rstrip('/')
    full_controller = name if name.startswith('/') else prefix + '/' + name
    full_action = full_controller.rstrip('/') + '/' + action.strip('/')
    if urdf == 'auto':
        candidates = sorted((root / 'urdf').glob('*.urdf'))
        candidates = [p for p in candidates if '.' not in p.stem]
        if len(candidates) > 1:
            raise ValueError('Multiple URDF files; pass robot_description_file explicitly')
        urdf = str(candidates[0]) if candidates else ''
    description = robot_description(urdf)
    central = None
    if (config / 'robot_config.yaml').is_file():
        from .central_config import load_central, adapt_central
        central = load_central(config / 'robot_config.yaml')
        adapted = adapt_central(central, joints, description)
        limits, hardware = adapted['limits'], adapted['hardware']
    else:
        limits = joint_limits(config / 'joint_limits.yaml', joints, description)
        hardware = read_yaml(config / 'advrf_hardware.yaml')
    mapping = hardware['joint_map']
    if (not isinstance(mapping, dict) or any(type(i) is not int or not 0 <= i <= 2147483647
                                           for i in mapping)
            or any(not isinstance(n, str) or not n for n in mapping.values())
            or len(set(mapping.values())) != len(mapping)):
        raise ValueError('joint_map must map unique nonnegative int32 IDs to unique joint names')
    reverse = {name: i for i, name in mapping.items()}
    try:
        ids = [reverse[j] for j in joints]
    except KeyError as error:
        raise ValueError(f'Missing hardware mapping: {error}') from error
    settings = config / 'advrf_controllers.yaml'
    nodes = {node: node_parameters(settings, node, namespace) for node in
             ('joint_command_controller_node', 'motor_power_node',
              'joint_trajectory_controller_node', 'joint_go_to_node')}
    configured_modes = [nodes[n].get('control_mode', 'position') for n in
                        ('joint_command_controller_node', 'motor_power_node')]
    if not control_mode and configured_modes[0] != configured_modes[1]:
        raise ValueError('Converter and motor-power control_mode must match')
    mode = control_mode or (central.get('control_mode', 'position') if central else configured_modes[0])
    if mode not in MODES or mode not in hardware.get('profiles', {}):
        raise ValueError(f'No configured profile for control mode {mode}')
    selected = []
    for joint in joints:
        values = hardware['profiles'][mode].get(joint, {}).get('gains')
        if not isinstance(values, list) or len(values) != 5:
            raise ValueError(f'{mode}/{joint}: expected five gains')
        values = [number(v, f'{mode}/{joint} gains') for v in values]
        if any(abs(v) > 3.402823466e38 for v in values):
            raise ValueError(f'{mode}/{joint}: gains exceed float32 range')
        selected.extend(values)
    command = selected[:]
    # Preserve the existing native bridge position/velocity wire permutation.
    if mode in ('position', 'velocity'):
        command = []
        for i in range(0, len(selected), 5):
            kp, ki, kd, unused3, unused4 = selected[i:i + 5]
            if unused3 != 0 or unused4 != 0:
                raise ValueError(f'{mode}: native position/velocity profile slots 4/5 must be zero')
            command.extend([kp, kd, 0.0, 0.0, ki])
    for node, values in nodes.items():
        for key, expected in limits.items():
            if key in values and values[key] != expected:
                raise ValueError(f'{node}: {key} must come from the shared joint limits')
        values.update(limits)
        if 'action_name' in values and values['action_name'] != full_action:
            raise ValueError(f'{node}: action_name must match moveit_controllers.yaml')
    nodes['joint_command_controller_node'].update(motor_ids=ids, joint_gains=command)
    # Converter currently receives the selected gains; drive mode is set by START.
    nodes['joint_command_controller_node']['control_mode'] = mode
    nodes['motor_power_node'].update(motor_ids=ids, control_mode=mode,
                                     start_joint_gains=selected)
    if nodes['motor_power_node'].get('start_gains'):
        raise ValueError('Use profiles instead of a second start_gains override')
    nodes['joint_trajectory_controller_node']['action_name'] = full_action
    nodes['joint_go_to_node']['action_name'] = full_action
    if central:
        adapted['parameters']['control_mode'] = mode
    return {'root': root, 'nodes': nodes, 'limits': limits, 'description': description,
            'moveit': document, 'action_name': full_action, 'control_mode': mode,
            'configuration_parameters': adapted['parameters'] if central else None,
            'configuration_file': str(config / 'robot_config.yaml') if central else '',
            'moveit_limits': adapted['moveit_limits'] if central else None}
