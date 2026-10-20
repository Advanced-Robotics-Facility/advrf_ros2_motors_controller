"""Open the rqt client only. Robot bringup owns all backend processes."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from advrf_ros2_motors_controller.panel_config import resolve_panel_config


def setup(context):
    joint_config = resolve_panel_config(LaunchConfiguration('joint_config').perform(context),
        os.environ.get('EC_CFG', ''), os.environ.get('ROS2_CFG', ''))
    return [ExecuteProcess(cmd=['ros2', 'run', 'rqt_gui', 'rqt_gui', '--force-discover',
        '--standalone', 'advrf_ros2_motors_controller.motor_panel.MotorPanel', '--args',
        '--joint-config', joint_config,
        '--namespace', LaunchConfiguration('namespace').perform(context)], output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('joint_config', default_value=''),
        DeclareLaunchArgument('namespace', default_value=''),
        OpaqueFunction(function=setup),
    ])
