"""Names and namespace validation for ROS tools."""
from rclpy.validate_namespace import validate_namespace


DEFAULT_ROBOT_NAMESPACE = '/advrf/robot'
ECAT_NAMESPACE_SUFFIX = '/ecat'


def resolve_namespace(explicit=None):
    namespace = DEFAULT_ROBOT_NAMESPACE if explicit is None else explicit
    namespace = '/' + namespace.strip().strip('/')
    validate_namespace(namespace)
    return namespace


def resolve_ecat_namespace(namespace=DEFAULT_ROBOT_NAMESPACE):
    value = resolve_namespace(namespace).rstrip('/') + ECAT_NAMESPACE_SUFFIX
    return resolve_namespace(value)
