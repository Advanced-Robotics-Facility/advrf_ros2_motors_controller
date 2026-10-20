#pragma once

namespace advrf_ros2_motors_controller::math
{
struct State
{
  double position;
  double velocity = 0;
  double acceleration = 0;
};

enum class Interpolation
{
  Linear,
  Cubic,
  Quintic
};

// Hermite interpolation, with derivatives expressed in seconds.
State sample_segment(const State &start, const State &end, double duration, double elapsed,
                     Interpolation interpolation);

// Minimum duration of a rest-to-rest quintic respecting continuous derivative limits.
double quintic_duration(double distance, double max_velocity, double max_acceleration);
} // namespace advrf_ros2_motors_controller::math
