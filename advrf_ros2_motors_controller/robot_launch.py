"""Launch the ROS stack from explicit robot configuration."""
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from .launch_config import ecat_remappings, robot_remappings
 
 
def stack_nodes(robot, namespace, transport, panel=True, bridge=False, joy=False):
    from .robot_config import node_parameters
    actions = []
    executables = {
        'joint_command_controller_node': 'converter_tx_motor_node',
        'joint_trajectory_controller_node': 'joint_trajectory_controller_node',
        'joint_go_to_node': 'joint_goal_manager_node',
    }
    if robot.get('configuration_parameters') is not None:
        actions.append(Node(package='advrf_ros2_motors_controller', executable='robot_config_node',
                            namespace=namespace, output='screen', parameters=[{
                                **robot['configuration_parameters'],
                                'configuration_file': robot['configuration_file'],
                                'robot_description': robot['description'],
                            }]))
    for configured_node, executable in executables.items():
        parameters = robot['nodes'][configured_node]
        actions.append(Node(package='advrf_ros2_motors_controller', executable=executable,
                            namespace=namespace, output='screen',
                            remappings=ecat_remappings(transport) + robot_remappings(namespace),
                            parameters=[parameters]))
    config = robot['root'] / 'config'
    if bridge:
        actions.append(Node(package='advrf_ros2_ecat', executable='bridge_ecat_client_node',
                            namespace=transport, output='screen',
                            parameters=[str(config / 'advrf_bridge.yaml')]))
    if panel:
        target = namespace.rstrip('/') + '/joint_goal_manager_node'
        actions.append(ExecuteProcess(cmd=[
            'ros2', 'run', 'rqt_gui', 'rqt_gui', '--force-discover', '--standalone',
            'advrf_ros2_motors_controller.motor_panel.MotorPanel', '--args',
            '--parameter-node', target, '--namespace', namespace,
            '--ecat-namespace', transport], output='screen'))
    if joy:
        path = config / 'joint_control.yaml'
        nodes = [('joy', 'joy_node', 'joy_node'),
                 ('advrf_ros2_io', 'joystick_input_node', 'joy_control_node'),
                 ('advrf_ros2_io', 'manual_joint_controller_node', 'joint_control_node')]
        for package, executable, configured_node in nodes:
            parameters = node_parameters(path, configured_node, namespace)
            if executable == 'manual_joint_controller_node':
                limits = robot['limits']
                joints = parameters.get('joints', limits['joints'])
                if not isinstance(joints, list) or not joints or any(
                        joint not in limits['joints'] for joint in joints) or len(set(joints)) != len(joints):
                    raise ValueError('joint_control.yaml: joints must be a nonempty list of unique robot joints')
                indices = [limits['joints'].index(joint) for joint in joints]
                parameters.update(joints=joints,
                                  lower_positions=[limits['position_lower'][i] for i in indices],
                                  upper_positions=[limits['position_upper'][i] for i in indices])
            actions.append(Node(package=package, executable=executable, namespace=namespace,
                                output='screen', parameters=[parameters],
                                remappings=ecat_remappings(transport) + robot_remappings(namespace)))
    return actions
 
 