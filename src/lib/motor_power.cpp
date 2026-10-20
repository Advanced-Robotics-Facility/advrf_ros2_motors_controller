#include "advrf_ros2_motors_controller/motor_power.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace advrf_ros2_motors_controller
{
ControlMode control_mode_from_string(const std::string &name)
{
  if (name == "position")
  {
    return ControlMode::Position;
  }
  if (name == "velocity")
  {
    return ControlMode::Velocity;
  }
  if (name == "impedance")
  {
    return ControlMode::Impedance;
  }
  if (name == "torque")
  {
    return ControlMode::Torque;
  }
  if (name == "current")
  {
    return ControlMode::Current;
  }
  throw std::invalid_argument("Unknown control_mode: " + name);
}

namespace middleware_msgs
{
advrf_interfaces::srv::ReplCmd::Request motor_start_request(int32_t board_id, ControlMode mode)
{
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
  advrf_interfaces::srv::ReplCmd::Request request;
  request.request.type = advrf_interfaces::msg::ReplCmdContent::CTRL_CMD;
  request.request.ctrl_cmd.type = advrf_interfaces::msg::CtrlCmd::CTRL_CMD_START;
  request.request.ctrl_cmd.board_id = board_id;
  request.request.ctrl_cmd.value = static_cast<uint8_t>(mode);
  // Leave gains unspecified: the middleware must omit the protobuf gains submessage.
  return request;
}

advrf_interfaces::srv::ReplCmd::Request motor_stop_request(int32_t board_id)
{
  if (board_id < 0)
  {
    throw std::invalid_argument("Negative motor board_id");
  }
  advrf_interfaces::srv::ReplCmd::Request request;
  request.request.type = advrf_interfaces::msg::ReplCmdContent::CTRL_CMD;
  request.request.ctrl_cmd.type = advrf_interfaces::msg::CtrlCmd::CTRL_CMD_STOP;
  request.request.ctrl_cmd.board_id = board_id;
  return request;
}
} // namespace middleware_msgs

struct MotorPowerClient::State
{
  using Service = advrf_interfaces::srv::ReplCmd;
  using Client = rclcpp::Client<Service>;
  using Clock = std::chrono::steady_clock;

  std::vector<int32_t> ids;
  Client::SharedPtr client;
  rclcpp::TimerBase::SharedPtr timer;
  std::optional<Client::FutureAndRequestId> pending;
  Completion completion;
  MotorPowerResult result;
  Clock::time_point since;
  double timeout;
  size_t index = 0;
  bool running = false;
  bool starting = false;
  ControlMode mode = ControlMode::Position;

  ~State()
  {
    if (pending)
    {
      client->remove_pending_request(*pending);
    }
  }

  void begin(bool start, ControlMode selected, Completion callback)
  {
    if (running)
    {
      throw std::logic_error("A motor power operation is already running");
    }
    if (!callback)
    {
      throw std::invalid_argument("Motor power completion callback is required");
    }
    if (start)
    {
      (void)middleware_msgs::motor_start_request(ids.front(), selected);
    }
    completion = std::move(callback);
    result = MotorPowerResult{};
    index = 0;
    starting = start;
    mode = selected;
    since = Clock::now();
    running = true;
  }

  void finish()
  {
    running = false;
    auto callback = std::move(completion);
    if (result.success)
    {
      result.message =
          starting ? "START acknowledged for all motors" : "STOP acknowledged for all motors";
    }
    callback(std::move(result));
  }

  void record(bool success, bool unknown, const std::string &message)
  {
    pending.reset();
    if (success)
    {
      result.acknowledged_ids.push_back(ids[index]);
    }
    else
    {
      result.success = false;
      result.outcome_unknown = result.outcome_unknown || unknown;
      if (!result.message.empty())
      {
        result.message += "; ";
      }
      result.message += "motor " + std::to_string(ids[index]) + ": " + message;
      if (unknown)
      {
        result.message += " (outcome unknown; no retry)";
      }
    }
    ++index;
    if (index == ids.size() || (starting && !success))
    {
      finish();
    }
    else
    {
      since = Clock::now();
    }
  }

  void tick()
  {
    if (!running)
    {
      return;
    }
    const bool expired = std::chrono::duration<double>(Clock::now() - since).count() >= timeout;
    if (pending)
    {
      if (pending->wait_for(std::chrono::seconds(0)) == std::future_status::ready)
      {
        Service::Response::SharedPtr response;
        try
        {
          response = pending->get();
        }
        catch (const std::exception &error)
        {
          record(false, true, error.what());
          return;
        }
        record(response && response->success, false,
               response ? response->msg : "Empty service response");
      }
      else if (expired)
      {
        client->remove_pending_request(*pending);
        record(false, true, "replCmd response timeout");
      }
      return;
    }
    if (!client->service_is_ready())
    {
      if (expired)
      {
        record(false, false, "replCmd unavailable; no command sent");
      }
      return;
    }
    const auto request = starting ? middleware_msgs::motor_start_request(ids[index], mode)
                                  : middleware_msgs::motor_stop_request(ids[index]);
    try
    {
      pending.emplace(client->async_send_request(std::make_shared<Service::Request>(request)));
      since = Clock::now();
    }
    catch (const std::exception &error)
    {
      record(false, true, error.what());
    }
  }
};

MotorPowerClient::MotorPowerClient(rclcpp::Node &node, const MotorsDefinition &motors,
                                   const std::vector<std::string> &joints, double timeout)
    : state_(std::make_shared<State>())
{
  if (joints.empty() || !std::isfinite(timeout) || timeout <= 0)
  {
    throw std::invalid_argument("MotorPowerClient requires joints and a positive timeout");
  }
  state_->timeout = timeout;
  for (const auto &joint : joints)
  {
    const auto id = motors.get_ecat_id(joint);
    if (std::find(state_->ids.begin(), state_->ids.end(), id) != state_->ids.end())
    {
      throw std::invalid_argument("Duplicate motor in MotorPowerClient");
    }
    state_->ids.push_back(id);
  }
  state_->client = node.create_client<State::Service>("replCmd");
  std::weak_ptr<State> weak = state_;
  state_->timer = node.create_wall_timer(std::chrono::milliseconds(10),
                                         [weak]()
                                         {
                                           if (auto state = weak.lock())
                                           {
                                             state->tick();
                                           }
                                         });
}

void MotorPowerClient::start(ControlMode mode, Completion completion)
{
  state_->begin(true, mode, std::move(completion));
}

void MotorPowerClient::stop(Completion completion)
{
  state_->begin(false, ControlMode::Position, std::move(completion));
}

bool MotorPowerClient::busy() const
{
  return state_->running;
}
} // namespace advrf_ros2_motors_controller
