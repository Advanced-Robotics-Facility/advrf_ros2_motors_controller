#include <gtest/gtest.h>
#include "advrf_ros2_motors_controller/command.hpp"
#include "advrf_ros2_motors_controller/trajectory_utils.hpp"
#include "advrf_ros2_motors_controller/controller_config.hpp"
#include <fstream>
#include <limits>
using namespace advrf_ros2_motors_controller;

trajectory_msgs::msg::JointTrajectory trajectory()
{
  trajectory_msgs::msg::JointTrajectory t;
  t.joint_names = {"a", "b"};
  trajectory_msgs::msg::JointTrajectoryPoint a, b;
  a.positions = {0, 2};
  b.positions = {4, 0};
  b.time_from_start.sec = 2;
  t.points = {a, b};
  return t;
}

TEST(Interpolation, LinearAndEndpointHold)
{
  auto t = trajectory();
  validate_trajectory(t);
  auto p = sample_trajectory(t, 1);
  EXPECT_EQ(p.positions, (std::vector<double>{2, 1}));
  EXPECT_EQ(p.velocities, (std::vector<double>{2, -1}));
  EXPECT_EQ(sample_trajectory(t, 3).velocities[0], 0);
  EXPECT_THROW(sample_trajectory(t, -1), std::invalid_argument);
}

TEST(Interpolation, CubicAndQuintic)
{
  auto t = trajectory();
  for (auto &p : t.points)
  {
    p.velocities = {0, 0};
  }

  validate_trajectory(t);
  EXPECT_NEAR(sample_trajectory(t, 1).positions[0], 2, 1e-12);
  EXPECT_NEAR(sample_trajectory(t, 1).velocities[0], 3, 1e-12);
  for (auto &p : t.points)
  {
    p.accelerations = {0, 0};
  }

  validate_trajectory(t);
  EXPECT_NEAR(sample_trajectory(t, 1).positions[0], 2, 1e-12);
  EXPECT_NEAR(sample_trajectory(t, 1).velocities[0], 3.75, 1e-12);
  EXPECT_NEAR(sample_trajectory(t, 1).accelerations[0], 0, 1e-12);
}

TEST(Interpolation, InvalidInput)
{
  auto t = trajectory();
  t.points[1].time_from_start.sec = 0;
  EXPECT_THROW(validate_trajectory(t), std::invalid_argument);
  t = trajectory();
  t.points[0].positions.pop_back();
  EXPECT_THROW(validate_trajectory(t), std::invalid_argument);
  t = trajectory();
  t.joint_names[1] = "a";
  EXPECT_THROW(validate_trajectory(t), std::invalid_argument);
  t = trajectory();
  t.points[0].positions[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(validate_trajectory(t), std::invalid_argument);
  t = trajectory();
  t.points[0].velocities = {0, 0};
  EXPECT_THROW(validate_trajectory(t), std::invalid_argument);
}

TEST(Motors, MappingAndCommands)
{
  MotorsDefinition motors;
  motors.declare(7, "a");
  motors.declare(2, "b");
  EXPECT_THROW(motors.declare(7, "c"), std::invalid_argument);
  EXPECT_THROW(motors.declare(8, "a"), std::invalid_argument);
  EXPECT_THROW(motors.get_ecat_id("unknown"), std::invalid_argument);
  auto p = sample_trajectory(trajectory(), 1);
  auto out = middleware_msgs::from_trajectory_point({"b", "a"}, p, motors, {1, 2, 3, 4, 5});
  EXPECT_EQ(out.ecat_id[0], 2);
  EXPECT_EQ(out.ecat_id[1], 7);
  EXPECT_FLOAT_EQ(out.pos_ref[0], 2);
  EXPECT_FLOAT_EQ(out.gain_4[0], 5);
  p.positions[0] = INFINITY;
  EXPECT_THROW(middleware_msgs::from_trajectory_point({"a", "b"}, p, motors),
               std::invalid_argument);
}

TEST(JointCommands, ConversionAndSharedLimits)
{
  sensor_msgs::msg::JointState msg;
  msg.name = {"b"};
  msg.position = {0.2};
  auto point = joint_command(msg);
  EXPECT_EQ(point.positions[0], 0.2);
  ControllerConfig config;
  config.joints = {"a", "b"};
  config.lower = {-1, -1};
  config.upper = {1, 1};
  config.velocity = {1, 1};
  config.acceleration = {5, 5};
  EXPECT_NO_THROW(config.check(point, msg.name));
  point.velocities = {2};
  EXPECT_THROW(config.check(point, msg.name), std::invalid_argument);
  msg.effort = {1};
  EXPECT_THROW(joint_command(msg), std::invalid_argument);
  msg.effort.clear();
  msg.name = {"b", "b"};
  msg.position = {0.1, 0.2};
  EXPECT_THROW(joint_command(msg), std::invalid_argument);
}

TEST(JointCommands, VelocityOnlyLimitsDoNotRelaxTrajectoryValidation)
{
  sensor_msgs::msg::JointState message;
  message.name = {"a"};
  message.velocity = {0.5};
  auto point = joint_command(message);
  ControllerConfig config;
  config.joints = {"a"};
  config.lower = {1}; // An absent position is not checked as an artificial zero.
  config.upper = {2};
  config.velocity = {1};
  config.acceleration = {5};
  EXPECT_NO_THROW(config.check_indexed(point, {0}, false));
  EXPECT_THROW(config.check(point, message.name), std::invalid_argument);
  point.velocities = {2};
  EXPECT_THROW(config.check_indexed(point, {0}, false), std::invalid_argument);
  message.velocity.clear();
  EXPECT_THROW(joint_command(message), std::invalid_argument);
}

TEST(Motors, JointStateMiddlewareConversion)
{
  MotorsDefinition motors;
  motors.declare(7, "a");
  motors.declare(2, "b");

  sensor_msgs::msg::JointState message;
  message.name = {"b", "a"};
  message.position = {0.25, -0.5};
  message.velocity = {0.75, -1.0};

  const auto output = middleware_msgs::from_joint_state(message, motors, {1, 2, 3, 4, 5});
  ASSERT_EQ(output.ecat_id.size(), 2u);
  EXPECT_EQ(output.ecat_id[0], 2);
  EXPECT_FLOAT_EQ(output.pos_ref[0], 0.25f);
  EXPECT_FLOAT_EQ(output.vel_ref[1], -1.0f);
  EXPECT_FLOAT_EQ(output.gain_4[1], 5.0f);

  message.position.clear();
  const auto velocity_only = middleware_msgs::from_joint_state(message, motors);
  EXPECT_FLOAT_EQ(velocity_only.pos_ref[0], 0.0f);
  EXPECT_FLOAT_EQ(velocity_only.vel_ref[0], 0.75f);
  message.position = {0.25, -0.5};
  message.velocity.clear();
  EXPECT_FLOAT_EQ(middleware_msgs::from_joint_state(message, motors).vel_ref[0], 0.0f);

  message.effort = {1.0, 1.0};
  EXPECT_THROW(middleware_msgs::from_joint_state(message, motors), std::invalid_argument);
  message.effort.clear();
  message.name = {"a", "a"};
  EXPECT_THROW(middleware_msgs::from_joint_state(message, motors), std::invalid_argument);
  message.name = {"a", "b"};
  message.position.clear();
  EXPECT_THROW(middleware_msgs::from_joint_state(message, motors), std::invalid_argument);
}

TEST(Motors, TrajectoryMiddlewareConversion)
{
  MotorsDefinition motors;
  motors.declare(7, "a");
  motors.declare(2, "b");

  const auto input = trajectory();
  validate_trajectory(input);
  const auto point = sample_trajectory(input, 1.0);
  const auto expected = middleware_msgs::from_trajectory_point(input.joint_names, point, motors);
  const auto actual = middleware_msgs::from_trajectory(input, 1.0, motors);
  EXPECT_EQ(actual, expected);
  EXPECT_THROW(middleware_msgs::from_trajectory(input, -1.0, motors), std::invalid_argument);
}

TEST(JointCommands, CachedIndicesAndLimits)
{
  ControllerConfig config;
  config.joints = {"a", "b"};
  config.lower = {-1, -2};
  config.upper = {1, 2};
  config.velocity = {1, 3};
  config.acceleration = {5, 10};

  const auto indices = config.joint_indices({"b", "a"});
  EXPECT_EQ(indices, (std::vector<size_t>{1, 0}));
  EXPECT_THROW(config.joint_indices({}), std::invalid_argument);
  EXPECT_THROW(config.joint_indices({"a", "a"}), std::invalid_argument);
  EXPECT_THROW(config.joint_indices({"unknown"}), std::invalid_argument);

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {2, 1};
  point.velocities = {3, 1};
  point.accelerations = {10, 5};
  EXPECT_NO_THROW(config.check_indexed(point, indices));
  EXPECT_NO_THROW(config.check(point, {"b", "a"}));
  EXPECT_THROW(config.check_indexed(point, {0, 1}), std::invalid_argument);

  point.positions[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(config.check_indexed(point, indices), std::invalid_argument);
  point.positions[0] = 2;
  point.velocities[0] = std::numeric_limits<double>::infinity();
  EXPECT_THROW(config.check_indexed(point, indices), std::invalid_argument);
  point.velocities[0] = 3;
  point.accelerations[0] = 11;
  EXPECT_THROW(config.check_indexed(point, indices), std::invalid_argument);
  point.accelerations.clear();
  point.velocities.clear();
  EXPECT_NO_THROW(config.check_indexed(point, indices));

  point.effort = {0, 0};
  EXPECT_THROW(config.check_indexed(point, indices), std::invalid_argument);
  point.effort.clear();
  point.velocities = {0};
  EXPECT_THROW(config.check_indexed(point, indices), std::invalid_argument);
  point.velocities.clear();
  point.accelerations = {0};
  EXPECT_THROW(config.check_indexed(point, indices), std::invalid_argument);
  point.accelerations.clear();
  EXPECT_THROW(config.check_indexed(point, {0, 99}), std::invalid_argument);
  point.positions.pop_back();
  EXPECT_THROW(config.check_indexed(point, indices), std::invalid_argument);

  // Changing a direct command's order must produce a fresh mapping.
  const auto subset = config.joint_indices({"a"});
  EXPECT_NO_THROW(config.check_indexed(point, config.joint_indices({"b"})));
  EXPECT_THROW(config.check_indexed(point, subset), std::invalid_argument);
}

TEST(Motors, ParameterIdsValidation)
{
  using advrf_ros2_motors_controller::MotorsDefinition;
  const auto motors = MotorsDefinition::from_ids({7, 1}, {"claw", "yaw"});
  EXPECT_EQ(motors.get_ecat_id("claw"), 7);
  EXPECT_EQ(motors.get_ecat_id("yaw"), 1);
  EXPECT_THROW(MotorsDefinition::from_ids({}, {}), std::invalid_argument);
  EXPECT_THROW(MotorsDefinition::from_ids({1}, {"a", "b"}), std::invalid_argument);
  EXPECT_THROW(MotorsDefinition::from_ids({1, 1}, {"a", "b"}), std::invalid_argument);
  EXPECT_THROW(MotorsDefinition::from_ids({1, 2}, {"a", "a"}), std::invalid_argument);
  EXPECT_THROW(MotorsDefinition::from_ids({-1}, {"a"}), std::invalid_argument);
  EXPECT_THROW(MotorsDefinition::from_ids({2147483648LL}, {"a"}), std::invalid_argument);
}
