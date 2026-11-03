#include <exception>
#include <std_srvs/srv/trigger.hpp>
#include <sys/types.h>

#include "advrf_ros2_interfaces/srv/repl_cmd.hpp"
#include "advrf_ros2_motors_controller/repl_cmd_manager.hpp"
#include "advrf_ros2_motors_controller/motor_types.hpp"
#include <algorithm>
#include "advrf_ros2_interfaces/srv/motor_power_request.hpp"

namespace advrf_ros2_motors_controller
{
class MotorPowerNode : public rclcpp::Node
{
  using Trigger = std_srvs::srv::Trigger;
  using Service = rclcpp::Service<Trigger>;
  using ServiceMotorPowerRequest = rclcpp::Service<advrf_ros2_interfaces::srv::MotorPowerRequest>;

public:
  MotorPowerNode() : Node("motor_power_node")
  {
    const auto joints = declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
    const auto map_path = declare_parameter<std::string>("robot_id_map", "");
    const auto ids = declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>{});
    const auto motors = ids.empty() ? MotorsDefinition::load(map_path, joints)
                                    : MotorsDefinition::from_ids(ids, joints);

    rcl_interfaces::msg::ParameterDescriptor gains_descriptor;
    gains_descriptor.read_only = true;
    gains_descriptor.description = "Optional START gains: pos_kp, pos_kd, tor_kp, tor_ki, tor_kd";
    const auto start_gains = declare_parameter<std::vector<double>>("start_gains", std::vector<double>{}, gains_descriptor);
    const auto start_joint_gains = declare_parameter<std::vector<double>>("start_joint_gains", std::vector<double>{}, gains_descriptor);

    rcl_interfaces::msg::ParameterDescriptor mode_descriptor;
    mode_descriptor.read_only = !start_joint_gains.empty();
    mode_descriptor.description = "Restart with a matching motor profile to change the drive mode";

    const auto mode = control_mode_from_string(declare_parameter<std::string>("control_mode", "position", mode_descriptor));
    if (joints.empty()) throw std::invalid_argument("At least one joint is required");
    if (!start_joint_gains.empty() && start_joint_gains.size() != 5 * joints.size())
      throw std::invalid_argument("start_joint_gains must contain five gains per joint");
    (void)ReplCmdBuilder::build_motor_start(0, static_cast<uint8_t>(mode), start_gains);
    for (size_t i = 0; i < joints.size(); ++i)
    {
      const auto id = motors.get_ecat_id(joints[i]);
      if (std::find(ids_.begin(), ids_.end(), id) != ids_.end())
        throw std::invalid_argument("Duplicate motor");
      ids_.push_back(id);
      auto gains = start_gains;
      if (gains.empty() && !start_joint_gains.empty())
        gains.assign(start_joint_gains.begin() + 5 * i, start_joint_gains.begin() + 5 * (i + 1));
      (void)ReplCmdBuilder::build_motor_start(id, static_cast<uint8_t>(mode), gains);
      gains_.push_back(std::move(gains));
    }
    repl_ = std::make_unique<ReplCmdManager>(*this, "replCmd", declare_parameter("motor_service_timeout", 3.0));
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

    power_request_ = create_service<advrf_ros2_interfaces::srv::MotorPowerRequest>("motor_power_request",
        [this, motors](ServiceMotorPowerRequest::SharedPtr service,
               std::shared_ptr<rmw_request_id_t> header,
               std::shared_ptr<advrf_ros2_interfaces::srv::MotorPowerRequest::Request> request)
        {

          using Power = advrf_ros2_interfaces::srv::MotorPowerRequest;
          auto respond = [service, header](bool success, bool unknown, const std::string &message)
          {
            Power::Response response;
            response.success = success;
            response.outcome_unknown = unknown;
            response.message = message;
            service->send_response(*header, response);
          };
          try
          {
            if (batch_running_ || repl_->busy())
              throw std::logic_error("A motor power operation is already running");
            if (request->action != Power::Request::ACTION_POWER_ON &&
                request->action != Power::Request::ACTION_POWER_OFF)
              throw std::invalid_argument("Unknown motor power action");
            std::vector<size_t> indices;
            if (request->joint != "all")
            {
              const auto id = motors.get_ecat_id(request->joint);
              const auto it = std::find(ids_.begin(), ids_.end(), id);
              if (it == ids_.end()) throw std::invalid_argument("Joint is not configured");
              indices.push_back(static_cast<size_t>(it - ids_.begin()));
            }
            const bool start = request->action == Power::Request::ACTION_POWER_ON;
            const auto selected = start
                ? control_mode_from_string(get_parameter("control_mode").as_string())
                : ControlMode::Position;
            begin(start, selected, respond, indices);
          }
          catch (const std::exception &error)
          {
            respond(false, false, error.what());
          }
        });


    if (declare_parameter("auto_start_motors", false))
    {
      begin(true, mode, [this](bool success, bool, const std::string &message){
            if (success)
            {
              RCLCPP_INFO(get_logger(), "%s", message.c_str());
            }else{
              RCLCPP_ERROR(get_logger(), "START failed: %s", message.c_str());
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
      begin(start, start ? control_mode_from_string(get_parameter("control_mode").as_string())
                         : ControlMode::Position,
            [respond](bool success, bool, const std::string &message) { respond(success, message); });
    }
    catch (const std::exception &error)
    {
      respond(false, error.what());
    }
  }

  // Power-specific sequencing stays here; the manager handles arbitrary REPL commands.
  void begin(bool start, ControlMode mode, std::function<void(bool, bool, const std::string &)> complete,
             const std::vector<size_t> &indices = {})
  {
    if (batch_running_ || repl_->busy())
      throw std::logic_error("A motor power operation is already running");
    batch_requests_.clear();
    batch_ids_.clear();
    for (size_t i = 0; i < ids_.size(); ++i)
    {
      if (!indices.empty() && std::find(indices.begin(), indices.end(), i) == indices.end()) continue;
      batch_ids_.push_back(ids_[i]);
      batch_requests_.push_back(start
          ? ReplCmdBuilder::build_motor_start(ids_[i], static_cast<uint8_t>(mode), gains_[i])
          : ReplCmdBuilder::build_motor_stop(ids_[i]));
    }
    batch_running_ = true;
    batch_starting_ = start;
    batch_success_ = true;
    batch_unknown_ = false;
    batch_index_ = 0;
    batch_message_.clear();
    batch_complete_ = std::move(complete);
    send_next();
  }

  void send_next()
  {
    repl_->send(nullptr, batch_requests_[batch_index_],
        [this](bool received, ReplCmd::Response::SharedPtr response)
        {
          const bool success = received && response->success;
          if (!success)
          {
            batch_success_ = false;
            batch_unknown_ = batch_unknown_ || !received;
            if (!batch_message_.empty()) batch_message_ += "; ";
            batch_message_ += "motor " + std::to_string(batch_ids_[batch_index_]) + ": " +
                (received ? response->msg : "replCmd failed or timed out (outcome unknown; no retry)");
          }
          ++batch_index_;
          // START stops at the first failure. STOP attempts every configured motor.
          if (batch_index_ == batch_requests_.size() || (batch_starting_ && !success))
          {
            batch_running_ = false;
            if (batch_success_) batch_message_ = batch_starting_
                ? "START acknowledged for selected motors" : "STOP acknowledged for selected motors";
            auto complete = std::move(batch_complete_);
            complete(batch_success_, batch_unknown_, batch_message_);
          }
          else send_next();
        });
  }

  std::unique_ptr<ReplCmdManager> repl_;
  std::vector<int32_t> ids_;
  std::vector<std::vector<double>> gains_;
  std::vector<ReplCmd::Request> batch_requests_;
  std::function<void(bool, bool, const std::string &)> batch_complete_;
  std::vector<int32_t> batch_ids_;
  bool batch_unknown_ = false;
  size_t batch_index_ = 0;
  bool batch_running_ = false;
  bool batch_starting_ = false;
  bool batch_success_ = true;
  std::string batch_message_;
  Service::SharedPtr start_;
  Service::SharedPtr stop_;

  ServiceMotorPowerRequest::SharedPtr power_request_;
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
