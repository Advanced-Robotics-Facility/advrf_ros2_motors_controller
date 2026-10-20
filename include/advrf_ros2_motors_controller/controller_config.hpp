#pragma once

#include "command.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <algorithm>
#include <set>

namespace advrf_ros2_motors_controller
{
// Shared by the action controller and the direct JointState controller.
struct ControllerConfig
{
  std::vector<std::string> joints;
  MotorsDefinition motors;
  std::vector<double> lower;
  std::vector<double> upper;
  std::vector<double> velocity;
  std::vector<double> acceleration;
  std::array<float, 5> gains{};

  static ControllerConfig declare(rclcpp::Node &node);

  size_t index(const std::string &name) const;

  // Resolve and validate names once when the command order changes.
  std::vector<size_t> joint_indices(const std::vector<std::string> &names) const;

  // Indices must come from joint_indices() for this configuration.
  void check_indexed(const trajectory_msgs::msg::JointTrajectoryPoint &point,
                     const std::vector<size_t> &indices) const;

  void check(const trajectory_msgs::msg::JointTrajectoryPoint &point,
             const std::vector<std::string> &names) const;
};

rclcpp::QoS declare_command_qos(rclcpp::Node &node);

// Commands require positions, optional velocities, no effort; never discard unsupported fields.
trajectory_msgs::msg::JointTrajectoryPoint
joint_command(const sensor_msgs::msg::JointState &message);
} // namespace advrf_ros2_motors_controller
