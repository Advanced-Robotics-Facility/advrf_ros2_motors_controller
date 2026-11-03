from advrf_ros2_motors_controller.launch_config import controller_launch


def generate_launch_description():
    return controller_launch('joint_limits', 'motor_power_manager_node', motor_power=True)
