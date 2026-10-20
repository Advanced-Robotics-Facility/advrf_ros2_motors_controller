#pragma once

#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace advrf_ros2_motors_controller
{
inline double time(const builtin_interfaces::msg::Duration &msg)
{
  return msg.sec + msg.nanosec * 1e-9;
}

inline double time(const trajectory_msgs::msg::JointTrajectoryPoint &point)
{
  return time(point.time_from_start);
}

void validate_trajectory(const trajectory_msgs::msg::JointTrajectory &trajectory);
// Linear, cubic Hermite or quintic Hermite depending on supplied derivatives.
// Validate once before calling in the cycle. Endpoint holds have zero derivatives.
trajectory_msgs::msg::JointTrajectoryPoint
sample_trajectory(const trajectory_msgs::msg::JointTrajectory &trajectory, double seconds);
} // namespace advrf_ros2_motors_controller
