#include "advrf_ros2_motors_controller/repl_cmd_manager.hpp"
#include <algorithm>
#include <map>

namespace advrf_ros2_motors_controller
{
ControlMode control_mode_from_string(const std::string &name)
{
  static const std::map<std::string, ControlMode> modes{
      {"position", ControlMode::Position},   {"velocity", ControlMode::Velocity},
      {"impedance", ControlMode::Impedance}, {"torque", ControlMode::Torque},
      {"current", ControlMode::Current},
  };

  const auto it = modes.find(name);
  if (it == modes.end())
  {
    throw std::invalid_argument("Unknown control_mode: " + name);
  }
  return it->second;
}

namespace ReplCmdBuilder
{
advrf_ros2_interfaces::srv::ReplCmd::Request build_motor_start(int32_t board_id, uint8_t raw_mode, const std::vector<double> &start_gains)
{
  const auto mode = static_cast<ControlMode>(raw_mode);
  if (board_id < 0)
  {
    throw std::invalid_argument("Negative motor board_id");
  }
  switch (mode)
  {
  case ControlMode::Position:
  case ControlMode::Velocity:
  case ControlMode::Impedance:
  case ControlMode::Torque:
  case ControlMode::Current:
    break;
  default:
    throw std::invalid_argument("Unsupported motor control mode");
  }
  advrf_ros2_interfaces::srv::ReplCmd::Request request;
  request.request.type = advrf_ros2_interfaces::msg::ReplCmdContent::CTRL_CMD;
  request.request.ctrl_cmd.type = advrf_ros2_interfaces::msg::CtrlCmd::CTRL_CMD_START;
  request.request.ctrl_cmd.board_id = board_id;
  request.request.ctrl_cmd.value = static_cast<uint8_t>(mode);
  if (!start_gains.empty())
  {
    if (start_gains.size() != 5 ||
        !std::all_of(start_gains.begin(), start_gains.end(),
                     [](double value) { return std::isfinite(value); }))
    {
      throw std::invalid_argument("start_gains must contain five finite values or be empty");
    }
    auto &gains = request.request.ctrl_cmd.gains;
    gains.type = static_cast<uint8_t>(mode);
    gains.pos_kp = start_gains[0];
    gains.pos_kd = start_gains[1];
    gains.tor_kp = start_gains[2];
    gains.tor_ki = start_gains[3];
    gains.tor_kd = start_gains[4];
  }
  // Empty start_gains leaves gains.type=0, requesting middleware gain preservation.
  return request;
}

advrf_ros2_interfaces::srv::ReplCmd::Request build_motor_stop(int32_t board_id)
{
  if (board_id < 0)
  {
    throw std::invalid_argument("Negative motor board_id");
  }
  advrf_ros2_interfaces::srv::ReplCmd::Request request;
  request.request.type = advrf_ros2_interfaces::msg::ReplCmdContent::CTRL_CMD;
  request.request.ctrl_cmd.type = advrf_ros2_interfaces::msg::CtrlCmd::CTRL_CMD_STOP;
  request.request.ctrl_cmd.board_id = board_id;
  return request;
}
} // namespace ReplCmdBuilder

} // namespace advrf_ros2_motors_controller
