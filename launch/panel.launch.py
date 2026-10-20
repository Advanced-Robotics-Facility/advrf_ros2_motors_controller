"""Launch the local rqt panel with optional existing GoTo and power nodes."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from advrf_ros2_motors_controller.launch_config import resolve_paths
from advrf_ros2_motors_controller.panel_config import resolve_panel_config


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    joint_config = resolve_panel_config(value('joint_config'),
        os.environ.get('EC_CFG', ''), os.environ.get('ROS2_CFG', ''))
    id_map = value('robot_id_map')
    if value('start_controllers') == 'true':
        joint_config, id_map = resolve_paths(joint_config, id_map,
            os.environ.get('EC_CFG', ''), os.environ.get('ROS2_CFG', ''), 'joint_config')
    launch_dir = Path(get_package_share_directory('advrf_ros2_motors_controller')) / 'launch'
    common = {'joint_config': joint_config, 'robot_id_map': id_map, 'namespace': value('namespace'),
              'control_mode': value('control_mode'), 'motor_profile': value('motor_profile'),
              'motor_profiles_dir': value('motor_profiles_dir')}
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_dir / 'joint.launch.py')),
            launch_arguments={**common, 'command_enabled': 'false'}.items(),
            condition=IfCondition(LaunchConfiguration('start_controllers'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_dir / 'go_to.launch.py')),
            launch_arguments={**common, 'trajectory_config': value('trajectory_config')}.items(),
            condition=IfCondition(LaunchConfiguration('start_controllers'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_dir / 'motors.launch.py')),
            launch_arguments={**common, 'control_mode': value('control_mode'),
                              'auto_start_motors': 'false', 'start_gains': value('start_gains')}.items(),
            condition=IfCondition(LaunchConfiguration('start_controllers'))),
        ExecuteProcess(cmd=['ros2', 'run', 'rqt_gui', 'rqt_gui', '--force-discover',
            '--standalone', 'advrf_ros2_motors_controller.motor_panel.MotorPanel', '--args',
            '--joint-config', joint_config, '--namespace', value('namespace')], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('joint_config', default_value=''),
        DeclareLaunchArgument('robot_id_map', default_value=''),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('trajectory_config', default_value=''),
        DeclareLaunchArgument('motor_profile', default_value=''),
        DeclareLaunchArgument('motor_profiles_dir', default_value=''),
        DeclareLaunchArgument('start_gains', default_value='',
            description='Optional explicit START gains [pos_kp, pos_kd, tor_kp, tor_ki, tor_kd]'),
        DeclareLaunchArgument('control_mode', default_value='position',
            choices=['position', 'impedance', 'velocity', 'torque', 'current']),
        DeclareLaunchArgument('start_controllers', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
