#pragma once

#include <advrf_ros2_interfaces/srv/repl_cmd.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace advrf_ros2_motors_controller
{
using ReplCmd = advrf_ros2_interfaces::srv::ReplCmd;
using ReplCmdClient = rclcpp::Client<ReplCmd>;
// The bool indicates transport completion; check response->success for command ACK.
using Callback = std::function<void(bool, std::shared_ptr<ReplCmd::Response>)>;

enum class ControlMode : uint8_t
{
  Position = advrf_ros2_interfaces::msg::Gains::POSITION,
  Velocity = advrf_ros2_interfaces::msg::Gains::VELOCITY,
  Impedance = advrf_ros2_interfaces::msg::Gains::IMPEDANCE,
  Torque = advrf_ros2_interfaces::msg::Gains::TORQUE,
  Current = advrf_ros2_interfaces::msg::Gains::CURRENT
};

ControlMode control_mode_from_string(const std::string &name);

struct TaskRepl
{
  std::chrono::steady_clock::time_point sent;
  std::shared_ptr<rmw_request_id_t> header_ptr;
  std::shared_ptr<ReplCmd::Request> request_ptr;
  std::optional<std::shared_ptr<ReplCmd::Response>> response_opt_ptr;
  std::optional<ReplCmdClient::FutureAndRequestId> pending_opt;
  Callback callback;
};

namespace ReplCmdBuilder
{
ReplCmd::Request build_motor_start(int32_t board_id, uint8_t mode,
                                  const std::vector<double> &start_gains = {});
ReplCmd::Request build_motor_stop(int32_t board_id);
}

// Use from a single-threaded executor. No blocking service wait or automatic retry.
class ReplCmdManager
{
public:
  ReplCmdManager(rclcpp::Node &node, const std::string &service_name, double timeout = 3.0)
      : timeout_(timeout), logger_(node.get_logger())
  {
    if (!std::isfinite(timeout_) || timeout_ <= 0)
      throw std::invalid_argument("ReplCmd timeout must be finite and positive");
    client_ = node.create_client<ReplCmd>(service_name);
    timer_ = node.create_wall_timer(std::chrono::milliseconds(10), [this]() { tick(); });
  }

  ~ReplCmdManager()
  {
    timer_->cancel();
    for (auto &task : pending_)
      if (task.pending_opt) client_->remove_pending_request(*task.pending_opt);
  }
  ReplCmdManager(const ReplCmdManager &) = delete;
  ReplCmdManager &operator=(const ReplCmdManager &) = delete;

  TaskRepl build(std::shared_ptr<rmw_request_id_t> header,
                 const ReplCmd::Request &request, Callback callback)
  {
    if (!callback) throw std::invalid_argument("ReplCmd callback is required");
    TaskRepl task;
    task.header_ptr = std::move(header);
    task.request_ptr = std::make_shared<ReplCmd::Request>(request);
    task.callback = std::move(callback);
    return task;
  }

  bool running() const { return rclcpp::ok(); }
  bool busy() const { return !pending_.empty(); }

  void send(TaskRepl task)
  {
    if (!running()) throw std::logic_error("ReplCmdManager is not running");
    if (!task.request_ptr || !task.callback || task.pending_opt)
      throw std::invalid_argument("Invalid ReplCmd task");
    task.sent = std::chrono::steady_clock::now();
    pending_.push_back(std::move(task));
  }

  void send(std::shared_ptr<rmw_request_id_t> header,
            const ReplCmd::Request &request, Callback callback)
  {
    send(build(std::move(header), request, std::move(callback)));
  }

private:
  ReplCmdClient::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<TaskRepl> pending_;
  double timeout_;
  rclcpp::Logger logger_;

  void tick()
  {
    // Callbacks may enqueue more work; invoke them only after iteration completes.
    std::vector<TaskRepl> completed;
    const auto now = std::chrono::steady_clock::now();
    for (auto it = pending_.begin(); it != pending_.end();)
    {
      bool done = false;
      try
      {
        if (it->pending_opt &&
            it->pending_opt->wait_for(std::chrono::seconds(0)) == std::future_status::ready)
        {
          it->response_opt_ptr = it->pending_opt->get();
          done = true;
        }
        else if (std::chrono::duration<double>(now - it->sent).count() >= timeout_)
        {
          if (it->pending_opt) client_->remove_pending_request(*it->pending_opt);
          done = true;
          RCLCPP_ERROR(logger_, "ReplCmd timeout; command outcome may be unknown");
        }
        else if (!it->pending_opt && client_->service_is_ready())
        {
          it->pending_opt.emplace(client_->async_send_request(it->request_ptr));
          it->sent = now;
        }
      }
      catch (const std::exception &error)
      {
        if (it->pending_opt) client_->remove_pending_request(*it->pending_opt);
        done = true;
        RCLCPP_ERROR(logger_, "ReplCmd error: %s", error.what());
      }
      if (done)
      {
        completed.push_back(std::move(*it));
        it = pending_.erase(it);
      }
      else ++it;
    }
    for (auto &task : completed)
    {
      auto response = task.response_opt_ptr.value_or(nullptr);
      const bool received = static_cast<bool>(response);
      try
      {
        task.callback(received, std::move(response));
      }
      catch (const std::exception &error)
      {
        RCLCPP_ERROR(logger_, "ReplCmd callback failed: %s", error.what());
      }
    }
  }
};
} // namespace advrf_ros2_motors_controller
