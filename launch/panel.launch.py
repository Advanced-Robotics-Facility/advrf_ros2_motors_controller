"""Open the rqt client only. Robot bringup owns all backend processes."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from advrf_ros2_motors_controller.panel_config import resolve_panel_config
from advrf_ros2_motors_controller.launch_config import config_arguments, config_argument, RobotNamespace


def setup(context):
    joint_limits = resolve_panel_config(LaunchConfiguration('joint_limits').perform(context),
        config_argument(context, 'ec_cfg'), config_argument(context, 'ros2_cfg'))
    return [ExecuteProcess(cmd=['ros2', 'run', 'rqt_gui', 'rqt_gui', '--force-discover',
        '--standalone', 'advrf_ros2_motors_controller.motor_panel.MotorPanel', '--args',
        '--joint-limits', joint_limits,
        '--namespace', LaunchConfiguration('namespace').perform(context)], output='screen')]


def generate_launch_description():
    return LaunchDescription(config_arguments() + [
        DeclareLaunchArgument('joint_limits', default_value=''),
        DeclareLaunchArgument('namespace', default_value=RobotNamespace()),
        OpaqueFunction(function=setup),
    ])
