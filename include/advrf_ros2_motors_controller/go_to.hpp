#pragma once

#include "controller_config.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace advrf_ros2_motors_controller
{
// Rest-to-rest motion. Unspecified joints remain at their measured position.
trajectory_msgs::msg::JointTrajectory
plan_go_to(const ControllerConfig &config, const std::vector<double> &start,
           const std::vector<std::string> &names, const std::vector<double> &positions,
           double velocity_scaling, double acceleration_scaling);
} // namespace advrf_ros2_motors_controller
