#include "advrf_ros2_motors_controller/controller_config.hpp"

namespace advrf_ros2_motors_controller
{
class JointCommandController : public rclcpp::Node
{
public:
  JointCommandController() : Node("joint_command_controller_node")
  {
    config_ = ControllerConfig::declare(*this);

    publisher_ = create_publisher<advrf_ros2_interfaces::msg::MotorTxPdoVector>("tx/motors", declare_command_qos(*this));
    subscription_ = create_subscription<sensor_msgs::msg::JointState>("joint_commands", rclcpp::QoS(1).reliable().durability_volatile(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr message)
        {
          forward(*message);
        });

    RCLCPP_INFO(get_logger(), "JointState-to-motor converter ready (%zu motors)",config_.joints.size());
  }

private:
  void forward(const sensor_msgs::msg::JointState &message)
  {
    try
    {
      const auto point = joint_command(message);
      if (message.name != joint_names_)
      {
        auto indices = config_.joint_indices(message.name);
        joint_names_ = message.name;
        joint_indices_ = std::move(indices);
      }
      config_.check_indexed(point, joint_indices_, false);
      publisher_->publish(middleware_msgs::from_joint_state(message, config_.motors, config_.gains));
    }
    catch (const std::exception &error)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Joint command rejected: %s", error.what());
    }
  }

  ControllerConfig config_;
  std::vector<std::string> joint_names_;
  std::vector<size_t> joint_indices_;
  rclcpp::Publisher<advrf_ros2_interfaces::msg::MotorTxPdoVector>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};
} // namespace advrf_ros2_motors_controller

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<advrf_ros2_motors_controller::JointCommandController>());
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("joint_command_controller_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
