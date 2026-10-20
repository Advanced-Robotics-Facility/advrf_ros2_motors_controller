#include "advrf_ros2_motors_controller/go_to.hpp"
#include "advrf_ros2_motors_controller/math/spline.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace advrf_ros2_motors_controller
{
trajectory_msgs::msg::JointTrajectory
plan_go_to(const ControllerConfig &config, const std::vector<double> &start,
           const std::vector<std::string> &names, const std::vector<double> &positions,
           double velocity_scaling, double acceleration_scaling)
{
  if (velocity_scaling == 0)
  {
    velocity_scaling = 1;
  }
  if (acceleration_scaling == 0)
  {
    acceleration_scaling = 1;
  }
  if (!std::isfinite(velocity_scaling) || velocity_scaling <= 0 || velocity_scaling > 1 ||
      !std::isfinite(acceleration_scaling) || acceleration_scaling <= 0 || acceleration_scaling > 1)
  {
    throw std::invalid_argument("GoTo scaling factors must be in (0, 1], or zero for defaults");
  }

  const auto canonical = config.joint_indices(config.joints);
  const auto selected = config.joint_indices(names);
  trajectory_msgs::msg::JointTrajectoryPoint initial;
  initial.positions = start;
  initial.velocities.assign(config.joints.size(), 0);
  initial.accelerations.assign(config.joints.size(), 0);
  config.check_indexed(initial, canonical);

  trajectory_msgs::msg::JointTrajectoryPoint target;
  target.positions = positions;
  config.check_indexed(target, selected);
  target = initial;
  for (size_t j = 0; j < selected.size(); ++j)
  {
    target.positions[selected[j]] = positions[j];
  }

  double duration = 0.001;
  for (size_t i = 0; i < start.size(); ++i)
  {
    duration =
        std::max(duration, math::quintic_duration(target.positions[i] - start[i],
                                                  config.velocity[i] * velocity_scaling,
                                                  config.acceleration[i] * acceleration_scaling));
  }
  // Round upward to nanoseconds so quantization cannot violate derivative bounds.
  const double nanoseconds = std::ceil(duration * 1e9);
  if (!std::isfinite(nanoseconds) ||
      nanoseconds >= static_cast<double>(std::numeric_limits<int32_t>::max()) * 1e9)
  {
    throw std::invalid_argument("GoTo duration exceeds ROS duration range");
  }
  const auto ticks = static_cast<int64_t>(nanoseconds);
  target.time_from_start.sec = static_cast<int32_t>(ticks / 1000000000);
  target.time_from_start.nanosec = static_cast<uint32_t>(ticks % 1000000000);

  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = config.joints;
  trajectory.points = {initial, target};
  return trajectory;
}
} // namespace advrf_ros2_motors_controller
