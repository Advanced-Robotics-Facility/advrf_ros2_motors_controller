"""Panel and all motor-control backends; master/middleware run separately."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    directory = Path(__file__).resolve().parent
    shared = ('namespace', 'joint_limits', 'robot_id_map', 'control_mode',
              'motor_profile', 'motor_profiles_dir')
    arguments = [
        DeclareLaunchArgument('namespace', default_value='', description='Robot namespace'),
        DeclareLaunchArgument('joint_limits', default_value='',
                              description='Defaults to ROS2_CFG or EC_CFG'),
        DeclareLaunchArgument('robot_id_map', default_value=''),
        DeclareLaunchArgument('control_mode', default_value='position',
                              choices=['position', 'impedance', 'velocity', 'torque', 'current']),
        DeclareLaunchArgument('motor_profile', default_value=''),
        DeclareLaunchArgument('motor_profiles_dir', default_value=''),
        DeclareLaunchArgument('trajectory_config', default_value=''),
        DeclareLaunchArgument('start_gains', default_value=''),
    ]

    def include(filename, names, **overrides):
        values = {name: LaunchConfiguration(name) for name in names}
        values.update(overrides)
        return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(directory / filename)),
            launch_arguments=values.items(),
        )])

    return LaunchDescription(arguments + [
        include('motors.launch.py', (*shared, 'start_gains'), auto_start_motors='false'),
        include('go_to.launch.py', (*shared, 'trajectory_config'),
                command_enabled='false', start_trajectory_controller='true'),
        include('joint.launch.py', shared, command_enabled='false'),
        include('panel.launch.py', ('namespace', 'joint_limits')),
    ])
