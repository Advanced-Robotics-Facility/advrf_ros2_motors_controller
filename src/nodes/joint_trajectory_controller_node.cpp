#include "advrf_ros2_motors_controller/controller_config.hpp"
#include "advrf_ros2_motors_controller/trajectory_utils.hpp"
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <algorithm>
#include <chrono>
#include <set>

namespace advrf_ros2_motors_controller
{
class JointTrajectoryController : public rclcpp::Node
{
  using Action = control_msgs::action::FollowJointTrajectory;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  using Point = trajectory_msgs::msg::JointTrajectoryPoint;

public:
  JointTrajectoryController() : Node("joint_trajectory_controller_node")
  {
    config_ = ControllerConfig::declare(*this, false);
    enabled_ = declare_parameter("command_enabled", true);
    enable_service_ = create_service<std_srvs::srv::SetBool>(
        "set_trajectory_enabled",
        [this](std_srvs::srv::SetBool::Request::SharedPtr request,
               std_srvs::srv::SetBool::Response::SharedPtr response)
        {
          if (active_ || reserved_)
          {
            response->success = false;
            response->message = "Cancel the active trajectory before changing command mode";
            return;
          }
          enabled_ = request->data;
          // Never resume a previous hold when returning from direct control.
          holding_ = false;
          response->success = true;
          response->message = enabled_ ? "Trajectory commands enabled" : "Trajectory commands disabled";
        });
    canonical_indices_ = config_.joint_indices(config_.joints);

    const double rate = declare_parameter("update_rate", 250.0);
    timeout_ = declare_parameter("feedback_timeout", 0.2);
    path_default_ = declare_parameter("path_tolerance", 0.1);
    goal_default_ = declare_parameter("goal_tolerance", 0.02);
    goal_time_default_ = declare_parameter("goal_time_tolerance", 2.0);

    if (!positive(rate) || rate > 10000 || !positive(timeout_) || !positive(path_default_) ||
        !positive(goal_default_) || !positive(goal_time_default_))
    {
      throw std::invalid_argument("Invalid rate, timeout or tolerance");
    }

    publisher_ = create_publisher<sensor_msgs::msg::JointState>(
        "joint_commands", rclcpp::QoS(1).reliable().durability_volatile());
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
        "rx/joint_states", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg)
        {
          receive(*msg);
        });

    server_ = rclcpp_action::create_server<Action>(
        this,
        declare_parameter<std::string>("action_name",
                                       "advrf_motors_controller/follow_joint_trajectory"),
        [this](const auto &, const std::shared_ptr<const Action::Goal> goal)
        {
          try
          {
            if (!enabled_)
            {
              throw std::invalid_argument("Trajectory commands disabled");
            }
            if (active_ || reserved_)
            {
              throw std::invalid_argument("Controller busy; cancel current goal first");
            }

            validate(*goal);
            reserved_ = true;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
          }
          catch (const std::exception &e)
          {
            RCLCPP_WARN(get_logger(), "Rejected goal: %s", e.what());
            return rclcpp_action::GoalResponse::REJECT;
          }
        },
        [](const std::shared_ptr<Handle>)
        {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<Handle> handle)
        {
          accept(handle);
        });

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate),
                               [this]()
                               {
                                 cycle();
                               });

    RCLCPP_INFO(get_logger(), "Ready: %zu motors at %.1f Hz", config_.joints.size(), rate);
  }

private:
  static bool positive(double v)
  {
    return std::isfinite(v) && v > 0;
  }

  bool fresh() const
  {
    return have_state_ &&
           std::chrono::duration<double>(std::chrono::steady_clock::now() - received_).count() <
               timeout_;
  }

  void receive(const sensor_msgs::msg::JointState &msg)
  {
    if (msg.name.size() != msg.position.size())
    {
      return;
    }
    std::set<std::string> names(msg.name.begin(), msg.name.end());
    if (names.size() != msg.name.size())
    {
      return;
    }

    Point next;
    for (const auto &joint : config_.joints)
    {
      auto it = std::find(msg.name.begin(), msg.name.end(), joint);
      if (it == msg.name.end())
      {
        return;
      }

      const auto i = static_cast<size_t>(it - msg.name.begin());
      if (!std::isfinite(msg.position[i]))
      {
        return;
      }

      next.positions.push_back(msg.position[i]);
    }

    actual_ = std::move(next);
    received_ = std::chrono::steady_clock::now();
    have_state_ = true;
  }

  std::vector<double> tolerances(const std::vector<control_msgs::msg::JointTolerance> &input,
                                 const std::vector<std::string> &names, double fallback) const
  {
    std::vector<double> out(names.size(), fallback);
    std::set<std::string> seen;
    for (const auto &t : input)
    {
      auto it = std::find(names.begin(), names.end(), t.name);
      if (it == names.end() || !seen.insert(t.name).second || !std::isfinite(t.position) ||
          (t.position < 0 && t.position != -1) || t.velocity != 0 || t.acceleration != 0)
      {
        throw std::invalid_argument("Only position tolerances supported; invalid joint/tolerance");
      }

      out[it - names.begin()] =
          t.position == 0 ? fallback : (t.position == -1 ? INFINITY : t.position);
    }

    return out;
  }

  void validate(const Action::Goal &goal) const
  {
    if (!fresh())
    {
      throw std::invalid_argument("No fresh complete joint feedback");
    }

    if (!goal.multi_dof_trajectory.points.empty() ||
        !goal.multi_dof_trajectory.joint_names.empty() || !goal.component_path_tolerance.empty() ||
        !goal.component_goal_tolerance.empty())
    {
      throw std::invalid_argument("Multi-DOF/component tolerances unsupported");
    }

    const auto &t = goal.trajectory;
    validate_trajectory(t);
    const auto indices = config_.joint_indices(t.joint_names);

    const rclcpp::Time stamp(t.header.stamp, get_clock()->get_clock_type());
    if (stamp.nanoseconds() != 0 && stamp < now())
    {
      throw std::invalid_argument("Old trajectory header stamp");
    }

    if (goal.goal_time_tolerance.sec < 0 || goal.goal_time_tolerance.nanosec >= 1000000000)
    {
      throw std::invalid_argument("Invalid goal_time_tolerance");
    }

    (void)tolerances(goal.path_tolerance, t.joint_names, path_default_);
    (void)tolerances(goal.goal_tolerance, t.joint_names, goal_default_);
    for (const auto &p : t.points)
    {
      config_.check_indexed(p, indices);
    }

    for (size_t i = 0; i < config_.joints.size(); ++i)
    {
      if (actual_.positions[i] < config_.lower[i] || actual_.positions[i] > config_.upper[i])
      {
        throw std::invalid_argument("Measured joint outside configured limits");
      }
    }
  }

  void accept(const std::shared_ptr<Handle> &handle)
  {
    reserved_ = false;
    active_ = handle;
    trajectory_ = handle->get_goal()->trajectory;
    goal_indices_ = config_.joint_indices(trajectory_.joint_names);
    const auto &goal = *handle->get_goal();
    path_ = tolerances(goal.path_tolerance, trajectory_.joint_names, path_default_);
    goal_ = tolerances(goal.goal_tolerance, trajectory_.joint_names, goal_default_);
    goal_time_ = time(goal.goal_time_tolerance);
    if (goal_time_ == 0)
    {
      goal_time_ = goal_time_default_;
    }

    start_ = rclcpp::Time(trajectory_.header.stamp, get_clock()->get_clock_type());
    if (start_.nanoseconds() == 0)
    {
      start_ = now();
    }

    last_clock_ = now();
    started_ = false;
    hold_.positions = actual_.positions;
    hold_.velocities.assign(config_.joints.size(), 0);
    holding_ = true;
  }

  Point measured_in_goal_order() const
  {
    Point out;
    out.positions.reserve(goal_indices_.size());
    for (const auto i : goal_indices_)
    {
      out.positions.push_back(actual_.positions[i]);
    }

    return out;
  }

  void finish(int code, const std::string &reason, bool canceled = false)
  {
    auto result = std::make_shared<Action::Result>();
    result->error_code = code;
    result->error_string = reason;
    // Hold the last measured position if available; stale feedback: retain the last command.
    if ((canceled || code != Action::Result::SUCCESSFUL) && fresh())
    {
      hold_ = actual_;
      hold_.velocities.assign(config_.joints.size(), 0);
    }

    if (canceled)
    {
      active_->canceled(result);
    }
    else if (code == Action::Result::SUCCESSFUL)
    {
      active_->succeed(result);
    }
    else
    {
      active_->abort(result);
    }

    active_.reset();
  }

  void publish_reference(const std::vector<std::string> &names, const Point &point)
  {
    sensor_msgs::msg::JointState message;
    message.header.stamp = now();
    message.name = names;
    message.position = point.positions;
    message.velocity = point.velocities;
    publisher_->publish(message);
  }

  void cycle()
  {
    if (!enabled_)
    {
      return;
    }
    try
    {
      if (!active_)
      {
        if (holding_ && fresh())
        {
          config_.check_indexed(hold_, canonical_indices_);
          publish_reference(config_.joints, hold_);
        }

        return;
      }

      if (active_->is_canceling())
      {
        finish(Action::Result::SUCCESSFUL, "Canceled; holding position", true);
        return;
      }

      if (!fresh())
      {
        finish(Action::Result::PATH_TOLERANCE_VIOLATED, "Joint feedback timeout");
        return;
      }

      const auto clock = now();
      if (clock < last_clock_)
      {
        finish(Action::Result::INVALID_GOAL, "ROS clock moved backwards");
        return;
      }

      last_clock_ = clock;
      const double elapsed = (clock - start_).seconds();
      if (elapsed < 0)
      {
        return;
      }

      if (!started_)
      {
        // Seed a delayed first point from the measured state at the actual start.
        if (time(trajectory_.points.front()) > 0)
        {
          auto initial = measured_in_goal_order();
          if (!trajectory_.points.front().velocities.empty())
          {
            initial.velocities.assign(goal_indices_.size(), 0);
          }

          if (!trajectory_.points.front().accelerations.empty())
          {
            initial.accelerations.assign(goal_indices_.size(), 0);
          }

          trajectory_.points.insert(trajectory_.points.begin(), initial);
        }

        started_ = true;
      }

      const auto desired = sample_trajectory(trajectory_, elapsed);
      config_.check_indexed(desired,
                            goal_indices_); // Includes spline overshoot at every emitted sample.
      const auto measured = measured_in_goal_order();
      const bool ended = elapsed >= time(trajectory_.points.back());
      bool reached = true;
      auto feedback = std::make_shared<Action::Feedback>();
      feedback->header.stamp = clock;
      feedback->joint_names = trajectory_.joint_names;
      feedback->desired = desired;
      feedback->actual = measured;
      feedback->actual.time_from_start = desired.time_from_start;
      for (size_t j = 0; j < goal_indices_.size(); ++j)
      {
        const double error = desired.positions[j] - measured.positions[j];
        feedback->error.positions.push_back(error);
        if (!ended && std::abs(error) > path_[j])
        {
          finish(Action::Result::PATH_TOLERANCE_VIOLATED,
                 "Path tolerance: " + trajectory_.joint_names[j]);
          return;
        }

        if (std::abs(error) > goal_[j])
        {
          reached = false;
        }
      }

      feedback->error.time_from_start = desired.time_from_start;
      publish_reference(trajectory_.joint_names, desired);

      // Maintain last emitted reference in canonical joint order.
      hold_.positions.resize(config_.joints.size());
      hold_.velocities.assign(config_.joints.size(), 0);
      for (size_t j = 0; j < goal_indices_.size(); ++j)
      {
        hold_.positions[goal_indices_[j]] = desired.positions[j];
      }

      active_->publish_feedback(feedback);
      if (ended && reached)
      {
        finish(Action::Result::SUCCESSFUL, "Goal reached");
      }
      else if (ended && elapsed > time(trajectory_.points.back()) + goal_time_)
      {
        finish(Action::Result::GOAL_TOLERANCE_VIOLATED, "Goal convergence timeout");
      }
    }
    catch (const std::exception &e)
    {
      if (active_)
      {
        finish(Action::Result::PATH_TOLERANCE_VIOLATED, e.what());
      }
      else
      {
        holding_ = false;
        RCLCPP_ERROR(get_logger(), "Hold stopped: %s", e.what());
      }
    }
  }

  bool enabled_ = true;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  ControllerConfig config_;
  std::vector<size_t> canonical_indices_;
  std::vector<size_t> goal_indices_;
  std::vector<double> path_;
  std::vector<double> goal_;

  double timeout_;
  double path_default_;
  double goal_default_;
  double goal_time_default_;
  double goal_time_{};

  bool have_state_ = false;
  bool reserved_ = false;
  bool started_ = false;
  bool holding_ = false;

  Point actual_;
  Point hold_;
  std::chrono::steady_clock::time_point received_;
  rclcpp::Time start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_clock_{0, 0, RCL_ROS_TIME};
  trajectory_msgs::msg::JointTrajectory trajectory_;
  std::shared_ptr<Handle> active_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace advrf_ros2_motors_controller

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<advrf_ros2_motors_controller::JointTrajectoryController>());
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("joint_trajectory_controller_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
