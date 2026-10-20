#pragma once

#include "motor_types.hpp"
#include "trajectory_utils.hpp"
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <advrf_interfaces/msg/motor_tx_pdo_vector.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <array>
#include <set>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace advrf_ros2_motors_controller::middleware_msgs
{
namespace detail
{
inline advrf_interfaces::msg::MotorTxPdoVector
from_joint_values(const std::vector<std::string> &joints, const std::vector<double> &positions,
                  const std::vector<double> &velocities, const MotorsDefinition &motors,
                  const std::array<float, 5> &gains = {})
{
  if (positions.size() != joints.size() ||
      (!velocities.empty() && velocities.size() != joints.size()))
  {
    throw std::invalid_argument("Command dimensions mismatch");
  }

  advrf_interfaces::msg::MotorTxPdoVector out;
  for (size_t i = 0; i < joints.size(); ++i)
  {
    const double position = positions[i];
    const double velocity = velocities.empty() ? 0.0 : velocities[i];
    for (double value : {position, velocity})
    {
      if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
      {
        throw std::invalid_argument("Motor reference outside finite float32 range");
      }
    }

    advrf_interfaces::msg::MotorTxPdo p;
    p.ecat_id = motors.get_ecat_id(joints[i]);
    p.pos_ref = static_cast<float>(position);
    p.vel_ref = static_cast<float>(velocity);
    const auto &selected_gains = motors.gains_for_joint(joints[i], gains);
    p.gain_0 = selected_gains[0];
    p.gain_1 = selected_gains[1];
    p.gain_2 = selected_gains[2];
    p.gain_3 = selected_gains[3];
    p.gain_4 = selected_gains[4];
    out.data.push_back(p);
  }

  return out;
}
} // namespace detail

inline advrf_interfaces::msg::MotorTxPdoVector
from_trajectory_point(const std::vector<std::string> &joints,
                      const trajectory_msgs::msg::JointTrajectoryPoint &point,
                      const MotorsDefinition &motors, const std::array<float, 5> &gains = {})
{
  return detail::from_joint_values(joints, point.positions, point.velocities, motors, gains);
}

inline advrf_interfaces::msg::MotorTxPdoVector
from_joint_state(const sensor_msgs::msg::JointState &message, const MotorsDefinition &motors,
                 const std::array<float, 5> &gains = {})
{
  const std::set<std::string> names(message.name.begin(), message.name.end());
  if (message.name.empty() || names.count("") || names.size() != message.name.size() ||
      !message.effort.empty())
  {
    throw std::invalid_argument("Expected unique joint names and no effort");
  }

  return detail::from_joint_values(message.name, message.position, message.velocity, motors, gains);
}

// Sample a previously validated trajectory, then convert its reference to motor commands.
inline advrf_interfaces::msg::MotorTxPdoVector
from_trajectory(const trajectory_msgs::msg::JointTrajectory &trajectory, double seconds,
                const MotorsDefinition &motors, const std::array<float, 5> &gains = {})
{
  return from_trajectory_point(trajectory.joint_names, sample_trajectory(trajectory, seconds),
                               motors, gains);
}
} // namespace advrf_ros2_motors_controller::middleware_msgs
