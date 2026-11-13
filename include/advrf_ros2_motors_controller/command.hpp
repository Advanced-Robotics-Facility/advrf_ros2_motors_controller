#pragma once

#include "motor_types.hpp"
#include "trajectory_utils.hpp"
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <advrf_ros2_interfaces/msg/motor_tx_pdo.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <array>
#include <vector>
#include <set>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace advrf_ros2_motors_controller::middleware_msgs
{
namespace detail
{
inline advrf_ros2_interfaces::msg::MotorTxPdo
from_joint_values(const std::vector<std::string> &joints, const std::vector<double> &positions,
                  const std::vector<double> &velocities, const MotorsDefinition &motors,
                  const std::array<float, 5> &gains = {})
{
  if (positions.size() != joints.size() ||
      (!velocities.empty() && velocities.size() != joints.size()))
  {
    throw std::invalid_argument("Command dimensions mismatch");
  }

  advrf_ros2_interfaces::msg::MotorTxPdo out;
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

    out.ecat_id.push_back(motors.get_ecat_id(joints[i]));
    out.pos_ref.push_back(static_cast<float>(position));
    out.vel_ref.push_back(static_cast<float>(velocity));
    const auto &selected_gains = motors.gains_for_joint(joints[i], gains);
    out.gain_0.push_back(selected_gains[0]);
    out.gain_1.push_back(selected_gains[1]);
    out.gain_2.push_back(selected_gains[2]);
    out.gain_3.push_back(selected_gains[3]);
    out.gain_4.push_back(selected_gains[4]);
    out.tor_ref.push_back(0);
    out.cur_ref.push_back(0);
    out.fault_ack.push_back(0);
    out.ts.push_back(0);
    out.op_idx_aux.push_back(0);
    out.aux.push_back(0);
  }

  return out;
}
} // namespace detail

inline advrf_ros2_interfaces::msg::MotorTxPdo
from_trajectory_point(const std::vector<std::string> &joints,
                      const trajectory_msgs::msg::JointTrajectoryPoint &point,
                      const MotorsDefinition &motors, const std::array<float, 5> &gains = {})
{
  return detail::from_joint_values(joints, point.positions, point.velocities, motors, gains);
}

inline advrf_ros2_interfaces::msg::MotorTxPdo
from_joint_state(const sensor_msgs::msg::JointState &message, const MotorsDefinition &motors,
                 const std::array<float, 5> &gains = {})
{
  const std::set<std::string> names(message.name.begin(), message.name.end());
  if (message.name.empty() || names.count("") || names.size() != message.name.size() ||
      !message.effort.empty() || (message.position.empty() && message.velocity.empty()))
  {
    throw std::invalid_argument("Expected unique joint names and no effort");
  }

  // The fixed PDO layout always carries pos_ref; velocity-only commands use zero.
  const auto positions = message.position.empty()
      ? std::vector<double>(message.name.size(), 0.0) : message.position;
  return detail::from_joint_values(message.name, positions, message.velocity, motors, gains);
}

// Sample a previously validated trajectory, then convert its reference to motor commands.
inline advrf_ros2_interfaces::msg::MotorTxPdo
from_trajectory(const trajectory_msgs::msg::JointTrajectory &trajectory, double seconds,
                const MotorsDefinition &motors, const std::array<float, 5> &gains = {})
{
  return from_trajectory_point(trajectory.joint_names, sample_trajectory(trajectory, seconds),
                               motors, gains);
}
} // namespace advrf_ros2_motors_controller::middleware_msgs
