#include "advrf_ros2_motors_controller/go_to.hpp"
#include "advrf_ros2_motors_controller/trajectory_utils.hpp"
#include <advrf_interfaces/action/go_to_joint_positions.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <set>
#include <limits>

namespace advrf_ros2_motors_controller
{
class JointGoTo : public rclcpp::Node
{
  using GoTo = advrf_interfaces::action::GoToJointPositions;
  using Follow = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<GoTo>;
  using FollowHandle = rclcpp_action::ClientGoalHandle<Follow>;
  using Clock = std::chrono::steady_clock;

public:
  JointGoTo() : Node("joint_go_to_node")
  {
    config_ = ControllerConfig::declare(*this, false);
    feedback_timeout_ = declare_parameter("feedback_timeout", 0.2);
    response_timeout_ = declare_parameter("response_timeout", 5.0);
    goal_time_tolerance_ = declare_parameter("goal_time_tolerance", 2.0);
    start_velocity_tolerance_ = declare_parameter("start_velocity_tolerance", 0.01);
    for (const double value :
         {feedback_timeout_, response_timeout_, goal_time_tolerance_, start_velocity_tolerance_})
    {
      if (!std::isfinite(value) || value <= 0)
      {
        throw std::invalid_argument("GoTo timeouts and start_velocity_tolerance must be positive");
      }
    }

    if (goal_time_tolerance_ >= std::numeric_limits<int32_t>::max())
    {
      throw std::invalid_argument("goal_time_tolerance exceeds the ROS duration range");
    }

    client_ = rclcpp_action::create_client<Follow>(
        this, declare_parameter<std::string>("action_name",
                                             "advrf_motors_controller/follow_joint_trajectory"));
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
        "rx/joint_states", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr message)
        {
          receive(*message);
        });
    server_ = rclcpp_action::create_server<GoTo>(
        this, "go_to_joint_positions",
        [this](const auto &, const std::shared_ptr<const GoTo::Goal> goal)
        {
          try
          {
            if (reserved_ || active_ || request_pending_ || downstream_)
            {
              throw std::invalid_argument("GoTo busy; cancel current goal first");
            }
            if (!client_->action_server_is_ready())
            {
              throw std::invalid_argument("Trajectory controller is unavailable");
            }
            if (!have_state_ || seconds_since(received_) >= feedback_timeout_)
            {
              throw std::invalid_argument("No fresh complete joint feedback");
            }
            if (!stationary_)
            {
              throw std::invalid_argument("GoTo requires a stationary starting state");
            }

            pending_ = plan_go_to(config_, positions_, goal->joint_names, goal->positions,
                                  goal->velocity_scaling, goal->acceleration_scaling);
            reserved_ = true;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
          }
          catch (const std::exception &error)
          {
            RCLCPP_WARN(get_logger(), "GoTo rejected: %s", error.what());
            return rclcpp_action::GoalResponse::REJECT;
          }
        },
        [this](const std::shared_ptr<GoalHandle>)
        {
          cancel_requested_ = true;
          cancel_downstream();
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle> handle)
        {
          execute(handle);
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(20),
                               [this]()
                               {
                                 watchdog();
                               });
    RCLCPP_INFO(get_logger(), "Ready for GoTo requests (%zu joints)", config_.joints.size());
  }

private:
  static double seconds_since(Clock::time_point stamp)
  {
    return std::chrono::duration<double>(Clock::now() - stamp).count();
  }

  void receive(const sensor_msgs::msg::JointState &message)
  {
    if (message.name.size() != message.position.size() ||
        (!message.velocity.empty() && message.velocity.size() != message.name.size()))
    {
      return;
    }
    const std::set<std::string> unique(message.name.begin(), message.name.end());
    if (unique.size() != message.name.size())
    {
      return;
    }

    std::vector<double> positions;
    positions.reserve(config_.joints.size());
    bool stationary = true;
    for (const auto &joint : config_.joints)
    {
      const auto it = std::find(message.name.begin(), message.name.end(), joint);
      if (it == message.name.end())
      {
        return;
      }
      const auto i = static_cast<size_t>(it - message.name.begin());
      if (!std::isfinite(message.position[i]))
      {
        return;
      }
      if (!message.velocity.empty())
      {
        if (!std::isfinite(message.velocity[i]))
        {
          return;
        }
        stationary = stationary && std::abs(message.velocity[i]) <= start_velocity_tolerance_;
      }
      positions.push_back(message.position[i]);
    }
    positions_ = std::move(positions);
    stationary_ = stationary;
    received_ = Clock::now();
    have_state_ = true;
  }

  void execute(const std::shared_ptr<GoalHandle> &handle)
  {
    reserved_ = false;
    active_ = handle;
    cancel_requested_ = false;
    cancel_sent_ = false;
    sent_ = Clock::now();
    execution_timeout_ = time(pending_.points.back()) + goal_time_tolerance_ + response_timeout_;
    request_pending_ = true;

    Follow::Goal goal;
    goal.trajectory = std::move(pending_);
    goal.goal_time_tolerance = rclcpp::Duration::from_seconds(goal_time_tolerance_);
    rclcpp_action::Client<Follow>::SendGoalOptions options;
    options.goal_response_callback = [this](FollowHandle::SharedPtr handle)
    {
      request_pending_ = false;
      downstream_ = handle;
      if (!handle)
      {
        finish(Follow::Result::INVALID_GOAL, "Trajectory controller rejected the goal");
      }
      else if (cancel_requested_ || !active_)
      {
        cancel_downstream();
      }
    };
    options.feedback_callback =
        [this](FollowHandle::SharedPtr, const std::shared_ptr<const Follow::Feedback> feedback)
    {
      if (!active_ || feedback->joint_names != config_.joints ||
          feedback->actual.positions.size() != target_positions_.size())
      {
        return;
      }
      auto report = std::make_shared<GoTo::Feedback>();
      report->joint_names = feedback->joint_names;
      report->actual_positions = feedback->actual.positions;
      report->position_errors.resize(feedback->actual.positions.size());
      // Report distance to the final target, rather than to the current spline sample.
      for (size_t i = 0; i < report->position_errors.size(); ++i)
      {
        report->position_errors[i] = target_positions_[i] - feedback->actual.positions[i];
      }
      active_->publish_feedback(report);
    };
    options.result_callback = [this](const FollowHandle::WrappedResult &wrapped)
    {
      downstream_.reset();
      if (!active_)
      {
        return;
      }
      if (!wrapped.result)
      {
        finish(Follow::Result::INVALID_GOAL, "Trajectory controller returned no result");
        return;
      }
      auto result = std::make_shared<GoTo::Result>();
      result->error_code = wrapped.result->error_code;
      result->message = wrapped.result->error_string;
      if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && result->error_code == 0)
      {
        active_->succeed(result);
      }
      else if (wrapped.code == rclcpp_action::ResultCode::CANCELED && active_->is_canceling())
      {
        active_->canceled(result);
      }
      else
      {
        if (result->error_code == 0)
        {
          result->error_code = Follow::Result::INVALID_GOAL;
        }
        active_->abort(result);
      }
      active_.reset();
    };
    target_positions_ = goal.trajectory.points.back().positions;
    try
    {
      client_->async_send_goal(goal, options);
    }
    catch (const std::exception &error)
    {
      request_pending_ = false;
      finish(Follow::Result::INVALID_GOAL, error.what());
    }
  }

  void cancel_downstream()
  {
    if (!downstream_ || cancel_sent_)
    {
      return;
    }
    try
    {
      client_->async_cancel_goal(downstream_);
      cancel_sent_ = true;
    }
    catch (const std::exception &error)
    {
      RCLCPP_ERROR(get_logger(), "Failed to forward cancellation: %s", error.what());
    }
  }

  void finish(int code, const std::string &message)
  {
    if (!active_)
    {
      return;
    }
    auto result = std::make_shared<GoTo::Result>();
    result->error_code = code;
    result->message = message;
    active_->abort(result);
    active_.reset();
  }

  void watchdog()
  {
    if (cancel_requested_)
    {
      cancel_downstream();
    }
    if (!active_)
    {
      return;
    }
    const auto elapsed = seconds_since(sent_);
    if ((request_pending_ && elapsed > response_timeout_) || elapsed > execution_timeout_)
    {
      cancel_requested_ = true;
      cancel_downstream();
      finish(Follow::Result::INVALID_GOAL, "Trajectory controller response/result timeout");
      // Keep busy until the downstream request is resolved: never orphan a running motion.
    }
  }

  ControllerConfig config_;
  std::vector<double> positions_;
  std::vector<double> target_positions_;
  trajectory_msgs::msg::JointTrajectory pending_;
  Clock::time_point received_;
  Clock::time_point sent_;
  double feedback_timeout_;
  double response_timeout_;
  double goal_time_tolerance_;
  double start_velocity_tolerance_;
  double execution_timeout_ = 0;
  bool have_state_ = false;
  bool stationary_ = false;
  bool reserved_ = false;
  bool request_pending_ = false;
  bool cancel_requested_ = false;
  bool cancel_sent_ = false;
  std::shared_ptr<GoalHandle> active_;
  FollowHandle::SharedPtr downstream_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp_action::Client<Follow>::SharedPtr client_;
  rclcpp_action::Server<GoTo>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace advrf_ros2_motors_controller

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<advrf_ros2_motors_controller::JointGoTo>());
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("joint_go_to_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
