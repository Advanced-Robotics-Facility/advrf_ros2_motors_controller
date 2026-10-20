#include "advrf_ros2_motors_controller/trajectory_utils.hpp"
#include "advrf_ros2_motors_controller/math/spline.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace advrf_ros2_motors_controller
{
void validate_trajectory(const trajectory_msgs::msg::JointTrajectory &t)
{
  const auto n = t.joint_names.size();
  std::set<std::string> names(t.joint_names.begin(), t.joint_names.end());
  if (!n || names.size() != n || names.count("") || t.points.empty())
  {
    throw std::invalid_argument("Empty trajectory or invalid joint names");
  }

  double previous = -1.0;
  const bool velocity = !t.points.front().velocities.empty();
  const bool acceleration = !t.points.front().accelerations.empty();
  for (const auto &p : t.points)
  {
    if (p.time_from_start.sec < 0 || p.time_from_start.nanosec >= 1000000000 ||
        time(p) <= previous || p.positions.size() != n ||
        p.velocities.size() != (velocity ? n : 0) ||
        p.accelerations.size() != (acceleration ? n : 0) || (acceleration && !velocity) ||
        !p.effort.empty())
    {
      throw std::invalid_argument("Invalid dimensions, derivatives, effort or point timing");
    }

    for (const auto *values : {&p.positions, &p.velocities, &p.accelerations})
    {
      for (double v : *values)
      {
        if (!std::isfinite(v))
        {
          throw std::invalid_argument("Nonfinite trajectory value");
        }
      }
    }

    previous = time(p);
  }
}

trajectory_msgs::msg::JointTrajectoryPoint
sample_trajectory(const trajectory_msgs::msg::JointTrajectory &t, double seconds)
{
  if (t.points.empty() || !std::isfinite(seconds) || seconds < 0)
  {
    throw std::invalid_argument("Invalid trajectory/sample time");
  }

  trajectory_msgs::msg::JointTrajectoryPoint out;
  const auto n = t.joint_names.size();
  out.positions.resize(n);
  out.velocities.assign(n, 0);
  out.accelerations.assign(n, 0);

  // Clamp time before conversion to a ROS duration.
  const double stamp = std::min(seconds, time(t.points.back()));
  out.time_from_start.sec = static_cast<int32_t>(stamp);
  out.time_from_start.nanosec = static_cast<uint32_t>((stamp - out.time_from_start.sec) * 1e9);
  if (seconds < time(t.points.front()) || seconds >= time(t.points.back()))
  {
    out.positions =
        seconds < time(t.points.front()) ? t.points.front().positions : t.points.back().positions;
    return out;
  }

  const auto next = std::upper_bound(t.points.begin(), t.points.end(), seconds,
                                     [](double s, const auto &p)
                                     {
                                       return s < time(p);
                                     });
  const auto &a = *(next - 1);
  const auto &b = *next;
  const double dt = time(b) - time(a);
  for (size_t j = 0; j < n; ++j)
  {
    const bool velocity = !a.velocities.empty();
    
    const bool acceleration = !a.accelerations.empty();
    const auto interpolation = acceleration ? math::Interpolation::Quintic
                               : velocity   ? math::Interpolation::Cubic
                                            : math::Interpolation::Linear;
    
    const math::State start{a.positions[j], velocity ? a.velocities[j] : 0, acceleration ? a.accelerations[j] : 0};
    const math::State end{b.positions[j], velocity ? b.velocities[j] : 0, acceleration ? b.accelerations[j] : 0};

    const auto sample = math::sample_segment(start, end, dt, seconds - time(a), interpolation);
    out.positions[j] = sample.position;
    out.velocities[j] = sample.velocity;
    out.accelerations[j] = sample.acceleration;
  }

  return out;
}
} // namespace advrf_ros2_motors_controller
