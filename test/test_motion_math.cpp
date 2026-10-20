#include <gtest/gtest.h>
#include "advrf_ros2_motors_controller/go_to.hpp"
#include "advrf_ros2_motors_controller/math/spline.hpp"
#include "advrf_ros2_motors_controller/trajectory_utils.hpp"
#include <cmath>
#include <limits>

using namespace advrf_ros2_motors_controller;

TEST(MotionMath, HermiteBoundaryConditions)
{
  const math::State a{1, 0.5, 0.25};
  const math::State b{-2, -0.75, -0.5};
  for (const auto type :
       {math::Interpolation::Linear, math::Interpolation::Cubic, math::Interpolation::Quintic})
  {
    const auto start = math::sample_segment(a, b, 2, 0, type);
    const auto end = math::sample_segment(a, b, 2, 2, type);
    EXPECT_NEAR(start.position, a.position, 1e-12);
    EXPECT_NEAR(end.position, b.position, 1e-12);
    if (type != math::Interpolation::Linear)
    {
      EXPECT_NEAR(start.velocity, a.velocity, 1e-12);
      EXPECT_NEAR(end.velocity, b.velocity, 1e-12);
    }
    if (type == math::Interpolation::Quintic)
    {
      EXPECT_NEAR(start.acceleration, a.acceleration, 1e-12);
      EXPECT_NEAR(end.acceleration, b.acceleration, 1e-12);
    }
  }
  EXPECT_THROW(math::sample_segment(a, b, 0, 0, math::Interpolation::Cubic), std::invalid_argument);
  EXPECT_THROW(math::sample_segment(a, b, 1, 2, math::Interpolation::Cubic), std::invalid_argument);
}

ControllerConfig motion_config()
{
  ControllerConfig config;
  config.joints = {"a", "b", "c"};
  config.lower = {-2, -3, -4};
  config.upper = {2, 3, 4};
  config.velocity = {1, 0.5, 2};
  config.acceleration = {2, 3, 1};
  return config;
}

TEST(MotionMath, SynchronizedQuinticRespectsLimitsBetweenEndpoints)
{
  const auto config = motion_config();
  for (const double scaling : {0.1, 0.5, 1.0})
  {
    const auto trajectory =
        plan_go_to(config, {-1, 2, 0.25}, {"b", "a"}, {-2, 1}, scaling, scaling);
    validate_trajectory(trajectory);
    ASSERT_EQ(trajectory.points.size(), 2u);
    EXPECT_EQ(trajectory.joint_names, config.joints);
    const double duration = time(trajectory.points.back());
    for (size_t i = 0; i <= 1000; ++i)
    {
      const auto point = sample_trajectory(trajectory, duration * (i / 1000.0));
      config.check(point, config.joints);
      EXPECT_DOUBLE_EQ(point.positions[2], 0.25);
      for (size_t j = 0; j < config.joints.size(); ++j)
      {
        EXPECT_LE(std::abs(point.velocities[j]), config.velocity[j] * scaling + 1e-12);
        EXPECT_LE(std::abs(point.accelerations[j]), config.acceleration[j] * scaling + 1e-12);
      }
    }
  }
}

TEST(MotionMath, GoToValidationAndStationaryGoal)
{
  const auto config = motion_config();
  const std::vector<double> start{0, 0, 0};
  EXPECT_THROW(plan_go_to(config, start, {"a", "a"}, {0, 0}, 1, 1), std::invalid_argument);
  EXPECT_THROW(plan_go_to(config, start, {"unknown"}, {0}, 1, 1), std::invalid_argument);
  EXPECT_THROW(plan_go_to(config, start, {"a"}, {}, 1, 1), std::invalid_argument);
  EXPECT_THROW(plan_go_to(config, start, {"a"}, {3}, 1, 1), std::invalid_argument);
  EXPECT_THROW(plan_go_to(config, start, {"a"}, {0}, 1.1, 1), std::invalid_argument);
  EXPECT_THROW(plan_go_to(config, start, {"a"}, {0}, 1, -1), std::invalid_argument);
  EXPECT_THROW(plan_go_to(config, {0}, {"a"}, {0}, 1, 1), std::invalid_argument);
  EXPECT_THROW(plan_go_to(config, start, {"a"}, {NAN}, 1, 1), std::invalid_argument);
  EXPECT_THROW(math::quintic_duration(1, 0, 1), std::invalid_argument);
  EXPECT_THROW(math::quintic_duration(INFINITY, 1, 1), std::invalid_argument);
  const auto stationary = plan_go_to(config, start, {"b"}, {0}, 0, 0);
  EXPECT_GT(time(stationary.points.back()), 0);
  EXPECT_EQ(stationary.points.front().positions, stationary.points.back().positions);
}
