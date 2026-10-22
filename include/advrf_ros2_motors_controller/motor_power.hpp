#pragma once

#include "motor_types.hpp"
#include <advrf_ros2_interfaces/srv/repl_cmd.hpp>
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>

namespace advrf_ros2_motors_controller
{
enum class ControlMode : uint8_t
{
  Position = advrf_ros2_interfaces::msg::Gains::POSITION,
  Velocity = advrf_ros2_interfaces::msg::Gains::VELOCITY,
  Impedance = advrf_ros2_interfaces::msg::Gains::IMPEDANCE,
  Torque = advrf_ros2_interfaces::msg::Gains::TORQUE,
  Current = advrf_ros2_interfaces::msg::Gains::CURRENT
};

ControlMode control_mode_from_string(const std::string &name);

struct MotorPowerResult
{
  bool success = true;
  bool outcome_unknown = false;
  std::vector<int32_t> acknowledged_ids;
  std::string message;
};

namespace middleware_msgs
{
// gains.type == 0 means that START must retain the drive's existing gains.
advrf_ros2_interfaces::srv::ReplCmd::Request motor_start_request(
    int32_t board_id, ControlMode mode, const std::vector<double> &start_gains = {});
advrf_ros2_interfaces::srv::ReplCmd::Request motor_stop_request(int32_t board_id);
} // namespace middleware_msgs

// Asynchronous, sequential calls. Use from a single-threaded executor.
// No automatic retry, implicit START, SET_GAINS, or hardware action in the constructor.
class MotorPowerClient
{
public:
  using Completion = std::function<void(MotorPowerResult)>;

  MotorPowerClient(rclcpp::Node &node, const MotorsDefinition &motors,
                   const std::vector<std::string> &joints, double timeout = 3.0,
                   const std::vector<double> &start_gains = {},
                   const std::vector<double> &start_joint_gains = {});
  void start(ControlMode mode, Completion completion);
  // STOP attempts every configured motor, even if an earlier STOP fails.
  void stop(Completion completion);
  bool busy() const;

private:
  struct State;
  std::shared_ptr<State> state_;
};
} // namespace advrf_ros2_motors_controller
