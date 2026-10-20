#include "advrf_ros2_motors_controller/math/spline.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace advrf_ros2_motors_controller::math
{
State sample_segment(const State &start, const State &end, double duration, double elapsed,
                     Interpolation interpolation)
{
  if (!std::isfinite(duration) || duration <= 0 || !std::isfinite(elapsed) || elapsed < 0 ||
      elapsed > duration)
  {
    throw std::invalid_argument("Invalid spline duration or sample time");
  }

  if (!std::isfinite(start.position) || !std::isfinite(end.position) ||
      !std::isfinite(start.velocity) || !std::isfinite(end.velocity) ||
      !std::isfinite(start.acceleration) || !std::isfinite(end.acceleration))
  {
    throw std::invalid_argument("Nonfinite spline boundary state");
  }

  const double u = elapsed / duration;
  const double delta = end.position - start.position;
  double c[6] = {start.position, delta, 0, 0, 0, 0};
  if (interpolation != Interpolation::Linear)
  {
    c[1] = start.velocity * duration;
    const double v = end.velocity * duration;
    c[2] = 3 * delta - 2 * c[1] - v;
    c[3] = -2 * delta + c[1] + v;
    if (interpolation == Interpolation::Quintic)
    {
      c[2] = start.acceleration * duration * duration / 2;
      const double d = delta - c[1] - c[2];
      const double e = v - c[1] - 2 * c[2];
      const double f = end.acceleration * duration * duration - 2 * c[2];
      c[3] = 10 * d - 4 * e + f / 2;
      c[4] = -15 * d + 7 * e - f;
      c[5] = 6 * d - 3 * e + f / 2;
    }
  }

  return {c[0] + u * (c[1] + u * (c[2] + u * (c[3] + u * (c[4] + u * c[5])))),
          (c[1] + u * (2 * c[2] + u * (3 * c[3] + u * (4 * c[4] + u * 5 * c[5])))) / duration,
          (2 * c[2] + u * (6 * c[3] + u * (12 * c[4] + u * 20 * c[5]))) / (duration * duration)};
}

double quintic_duration(double distance, double max_velocity, double max_acceleration)
{
  if (!std::isfinite(distance) || !std::isfinite(max_velocity) || max_velocity <= 0 ||
      !std::isfinite(max_acceleration) || max_acceleration <= 0)
  {
    throw std::invalid_argument("Invalid quintic distance or limits");
  }

  // s(u) = 10u^3 - 15u^4 + 6u^5:
  // max |s'| = 15/8, max |s''| = 10/sqrt(3), for u in [0, 1].
  const double d = std::abs(distance);
  const double duration =
      std::max(1.875 * d / max_velocity, std::sqrt((10 / std::sqrt(3.0)) * d / max_acceleration));
  if (!std::isfinite(duration))
  {
    throw std::invalid_argument("Quintic duration overflow");
  }
  return duration;
}
} // namespace advrf_ros2_motors_controller::math
