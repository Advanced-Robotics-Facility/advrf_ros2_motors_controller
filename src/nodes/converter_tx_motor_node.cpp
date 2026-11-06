#include "advrf_ros2_motors_controller/controller_config.hpp"
#include <advrf_ros2_interfaces/srv/set_joint_gains.hpp>
#include <cmath>
#include <limits>
#include <optional>
#include <chrono>

namespace advrf_ros2_motors_controller
{

class JointCommandController : public rclcpp::Node
{
public:
  JointCommandController() : Node("converter_tx_motor_node")
  {
    config_ = ControllerConfig::declare(*this);
    rcl_interfaces::msg::ParameterDescriptor rate_descriptor;
    rate_descriptor.read_only = true;
    const double publish_rate = declare_parameter("publish_rate", 250.0, rate_descriptor);
    if (!std::isfinite(publish_rate) || publish_rate <= 0 || publish_rate > 10000)
      throw std::invalid_argument("publish_rate must be finite and in (0, 10000] Hz");
    const auto mask = declare_parameter<std::vector<bool>>(
        "gain_mask", {true, true, true, true, true});
    if (mask.size() != 5)
      throw std::invalid_argument("gain_mask must contain exactly five booleans");
    mask_callback_ = add_on_set_parameters_callback(
        [](const std::vector<rclcpp::Parameter> &parameters)
        {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          for (const auto &parameter : parameters)
          {
            if (parameter.get_name() == "gain_mask" &&
                (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL_ARRAY ||
                 parameter.as_bool_array().size() != 5))
            {
              result.successful = false;
              result.reason = "gain_mask must contain exactly five booleans";
            }
          }
          return result;
        });
    gains_service_ = create_service<advrf_ros2_interfaces::srv::SetJointGains>(
        "set_joint_gains",
        [this](const std::shared_ptr<advrf_ros2_interfaces::srv::SetJointGains::Request> request,
               std::shared_ptr<advrf_ros2_interfaces::srv::SetJointGains::Response> response)
        {
          try
          {

            (void)config_.index(request->joint_name);
            auto gains = config_.motors.gains_for_joint(request->joint_name, config_.gains);
            const auto mask = get_parameter("gain_mask").as_bool_array();
            for (size_t i = 0; i < gains.size(); ++i)
            {
              const auto value = request->gains[i];
              if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
              {
                throw std::invalid_argument("Gains must be finite and representable as float32");
              }
              if (mask[i])
                gains[i] = static_cast<float>(value);
            }
            config_.motors.set_gains(request->joint_name, gains);
            response->success = true;
            response->message = "Gains updated; applied on the next publication";
          }
          catch (const std::exception &error)
          {
            response->success = false;
            response->message = error.what();
          }
        });

    publisher_ = create_publisher<advrf_ros2_interfaces::msg::MotorTxPdoVector>("tx/motors", declare_command_qos(*this));
    subscription_ = create_subscription<sensor_msgs::msg::JointState>("joint_commands", rclcpp::QoS(1).reliable().durability_volatile(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr message)
        {
          forward(*message);
        });

    publish_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate), [this]()
        {
          if (last_command_)
            publisher_->publish(middleware_msgs::from_joint_state(
                *last_command_, config_.motors, config_.gains));
        });
    RCLCPP_INFO(get_logger(), "JointState-to-motor converter ready (%zu motors, %.1f Hz)",
                config_.joints.size(), publish_rate);
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
      auto converted = middleware_msgs::from_joint_state(message, config_.motors, config_.gains);
      last_command_ = message;
      publisher_->publish(converted);
    }
    catch (const std::exception &error)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Joint command rejected: %s", error.what());
    }
  }

  ControllerConfig config_;
  std::optional<sensor_msgs::msg::JointState> last_command_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr mask_callback_;
  rclcpp::Service<advrf_ros2_interfaces::srv::SetJointGains>::SharedPtr gains_service_;
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
    RCLCPP_FATAL(rclcpp::get_logger("converter_tx_motor_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
