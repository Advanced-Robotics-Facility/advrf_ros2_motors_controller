"""Standalone ADVRF controls, with optional model, bridge, joystick and RViz."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from advrf_ros2_motors_controller.robot_config import load_robot
from advrf_ros2_motors_controller.robot_launch import stack_nodes


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    def enabled(name):
        return value(name).lower() == 'true'
    namespace = '/' + value('namespace').strip('/')
    transport = value('ecat_namespace') or namespace.rstrip('/') + '/ecat'
    robot = load_robot(value('config_dir'), namespace, value('robot_description_file'),
                       value('control_mode'))
    actions = stack_nodes(robot, namespace, transport, enabled('use_panel'),
                          enabled('start_bridge'), enabled('use_joy'))
    if robot['description']:
        actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                            namespace=namespace, output='screen',
                            parameters=[{'robot_description': robot['description']}],
                            remappings=[
                                ('joint_states', transport.rstrip('/') + '/rx/joint_states'),
                                ('robot_description', '/robot_description'),
                            ]))
    if enabled('use_rviz'):
        rviz = value('rviz_config') or str(robot['root'] / 'rviz' / 'miranda.rviz')
        if not robot['description']:
            raise ValueError('RViz robot visualization requires robot_description_file')
        actions.append(Node(package='rviz2', executable='rviz2', output='screen',
                            arguments=['-d', rviz]))
    return actions


def generate_launch_description():
    defaults = {
        'config_dir': '', 'namespace': '/advrf/robot', 'ecat_namespace': '',
        'robot_description_file': 'auto', 'control_mode': '',
        'use_panel': 'true', 'start_bridge': 'false', 'use_joy': 'false',
        'use_rviz': 'false', 'rviz_config': '',
    }
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
                             + [OpaqueFunction(function=setup)])
