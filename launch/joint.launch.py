from advrf_ros2_motors_controller.launch_config import controller_launch


def generate_launch_description():
    return controller_launch('joint_limits', 'converter_tx_motor_node')
