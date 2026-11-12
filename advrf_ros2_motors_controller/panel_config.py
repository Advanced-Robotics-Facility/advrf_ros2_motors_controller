"""Validate joint parameters received from ROS and measured feedback."""
import math


GET_PARAMETERS_SERVICE = 'get_parameters'
JOINT_PARAMETER_NAMES = (
    'joints',
    'position_lower',
    'position_upper',
    'max_velocity',
)
TRAJECTORY_PARAMETER_NAMES = (*JOINT_PARAMETER_NAMES, 'max_acceleration')


def decode_joint_parameters(values, names=JOINT_PARAMETER_NAMES):
    if (
        len(values) != len(names)
        or values[0].type != 9
        or any(value.type != 8 for value in values[1:])
    ):
        raise ValueError('Controller returned missing or incorrectly typed limits')

    parameters = {names[0]: list(values[0].string_array_value)}
    parameters.update({
        name: list(value.double_array_value)
        for name, value in zip(names[1:], values[1:])
    })
    validate_joint_parameters(parameters)
    return parameters


def validate_joint_parameters(parameters):
    names = parameters['joints']

    if (
        not names
        or any(not isinstance(name, str) or not name for name in names)
        or len(set(names)) != len(names)
    ):
        raise ValueError('Expected unique nonempty joint names')

    for key, values in parameters.items():
        if key == 'joints':
            continue

        if len(values) != len(names) or any(not math.isfinite(value) for value in values):
            raise ValueError(f'Invalid {key}: expected one finite value per joint')

        if key in ('max_velocity', 'max_acceleration') and any(value <= 0 for value in values):
            raise ValueError(f'{key} must be positive')

    if any(lo > hi for lo, hi in zip(parameters['position_lower'], parameters['position_upper'])):
        raise ValueError('Invalid position limits')


def measured_positions(names, positions, joints):
    if len(names) != len(positions) or len(set(names)) != len(names):
        raise ValueError('Malformed JointState')

    values = dict(zip(names, positions))
    result = [values[name] for name, _, _ in joints]

    if not all(math.isfinite(value) for value in result):
        raise ValueError('Nonfinite feedback')

    return result
