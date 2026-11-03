"""Launch the ROS stack from explicit robot configuration."""
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from .launch_config import ecat_remappings


def stack_nodes(robot, namespace, transport, panel=True, bridge=False, joy=False):
    from .robot_config import node_parameters
    actions = []
    if robot.get('configuration_parameters') is not None:
        actions.append(Node(package='advrf_ros2_motors_controller', executable='robot_config_node',
                            namespace=namespace, output='screen', parameters=[{
                                **robot['configuration_parameters'],
                                'configuration_file': robot['configuration_file'],
                                'robot_description': robot['description'],
                            }]))
    for executable, parameters in robot['nodes'].items():
        actions.append(Node(package='advrf_ros2_motors_controller', executable=executable,
                            namespace=namespace, output='screen',
                            remappings=ecat_remappings(transport), parameters=[parameters]))
    config = robot['root'] / 'config'
    if bridge:
        actions.append(Node(package='advrf_ros2_ecat', executable='advrf_ros2_ecat',
                            namespace=transport, output='screen',
                            parameters=[str(config / 'advrf_bridge.yaml')]))
    if panel:
        target = namespace.rstrip('/') + '/joint_go_to_node'
        actions.append(ExecuteProcess(cmd=[
            'ros2', 'run', 'rqt_gui', 'rqt_gui', '--force-discover', '--standalone',
            'advrf_ros2_motors_controller.motor_panel.MotorPanel', '--args',
            '--parameter-node', target, '--namespace', namespace,
            '--ecat-namespace', transport], output='screen'))
    if joy:
        path = config / 'joint_control.yaml'
        for package, executable in [('joy', 'joy_node'), ('advrf_ros2_io', 'joy_control_node'),
                                    ('advrf_ros2_io', 'joint_control_node')]:
            parameters = node_parameters(path, executable, namespace)
            if executable == 'joint_control_node':
                parameters.update(joints=robot['limits']['joints'],
                                  lower_positions=robot['limits']['position_lower'],
                                  upper_positions=robot['limits']['position_upper'])
            actions.append(Node(package=package, executable=executable, namespace=namespace,
                                output='screen', parameters=[parameters],
                                remappings=ecat_remappings(transport)))
    return actions
