#include "advrf_ros2_motors_controller/motor_power.hpp"
#include <std_srvs/srv/trigger.hpp>

namespace advrf_ros2_motors_controller
{
class MotorPowerNode : public rclcpp::Node
{
  using Trigger = std_srvs::srv::Trigger;
  using Service = rclcpp::Service<Trigger>;

public:
  MotorPowerNode() : Node("motor_power_node")
  {
    const auto joints =
        declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
    const auto motors =
        MotorsDefinition::load(declare_parameter<std::string>("robot_id_map", ""), joints);
    rcl_interfaces::msg::ParameterDescriptor gains_descriptor;
    gains_descriptor.read_only = true;
    gains_descriptor.description = "Optional START gains: pos_kp, pos_kd, tor_kp, tor_ki, tor_kd";
    const auto start_gains = declare_parameter<std::vector<double>>(
        "start_gains", std::vector<double>{}, gains_descriptor);
    const auto start_joint_gains = declare_parameter<std::vector<double>>(
        "start_joint_gains", std::vector<double>{}, gains_descriptor);
    rcl_interfaces::msg::ParameterDescriptor mode_descriptor;
    mode_descriptor.read_only = !start_joint_gains.empty();
    mode_descriptor.description = "Restart with a matching motor profile to change the drive mode";
    const auto mode = control_mode_from_string(
        declare_parameter<std::string>("control_mode", "position", mode_descriptor));
    power_ = std::make_unique<MotorPowerClient>(*this, motors, joints,
        declare_parameter("motor_service_timeout", 3.0), start_gains, start_joint_gains);

    const auto configured_gains = get_parameter("start_gains").as_double_array();
    RCLCPP_INFO(get_logger(), "START configuration: mode=%s (0x%02X), gains=%s",
                get_parameter("control_mode").as_string().c_str(), static_cast<unsigned>(mode),
                configured_gains.empty() ? (get_parameter("start_joint_gains").as_double_array().empty()
                    ? "unspecified (middleware preservation required)"
                    : get_parameter("start_joint_gains").value_to_string().c_str())
                                         : get_parameter("start_gains").value_to_string().c_str());

    start_ = create_service<Trigger>("start_motors",
                                     [this](Service::SharedPtr service,
                                            std::shared_ptr<rmw_request_id_t> header,
                                            std::shared_ptr<Trigger::Request>)
                                     {
                                       command(service, header, true);
                                     });
    stop_ = create_service<Trigger>("stop_motors",
                                    [this](Service::SharedPtr service,
                                           std::shared_ptr<rmw_request_id_t> header,
                                           std::shared_ptr<Trigger::Request>)
                                    {
                                      command(service, header, false);
                                    });

    if (declare_parameter("auto_start_motors", false))
    {
      power_->start(mode,
                    [this](MotorPowerResult result)
                    {
                      if (result.success)
                      {
                        RCLCPP_INFO(get_logger(), "%s", result.message.c_str());
                      }
                      else
                      {
                        RCLCPP_ERROR(get_logger(), "START failed: %s", result.message.c_str());
                      }
                    });
    }
    RCLCPP_INFO(get_logger(), "Motor power services ready in namespace %s", get_namespace());
  }

private:
  void command(const Service::SharedPtr &service, std::shared_ptr<rmw_request_id_t> header,
               bool start)
  {
    auto respond = [service, header](bool success, const std::string &message)
    {
      Trigger::Response response;
      response.success = success;
      response.message = message;
      try
      {
        service->send_response(*header, response);
      }
      catch (const std::exception &error)
      {
        RCLCPP_ERROR(rclcpp::get_logger("motor_power_node"), "Service reply failed: %s",
                     error.what());
      }
    };
    try
    {
      auto complete = [respond](MotorPowerResult result)
      {
        respond(result.success, result.message);
      };
      if (start)
      {
        power_->start(control_mode_from_string(get_parameter("control_mode").as_string()),
                      complete);
      }
      else
      {
        power_->stop(complete);
      }
    }
    catch (const std::exception &error)
    {
      respond(false, error.what());
    }
  }

  std::unique_ptr<MotorPowerClient> power_;
  Service::SharedPtr start_;
  Service::SharedPtr stop_;
};
} // namespace advrf_ros2_motors_controller

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<advrf_ros2_motors_controller::MotorPowerNode>());
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("motor_power_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
