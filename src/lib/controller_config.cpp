#include "advrf_ros2_motors_controller/controller_config.hpp"

namespace advrf_ros2_motors_controller
{
ControllerConfig ControllerConfig::declare(rclcpp::Node &node)
{
  ControllerConfig config;
  config.joints =
      node.declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
  auto id_map = node.declare_parameter<std::string>("robot_id_map", "");
  // Keep the former parameter working for existing direct node invocations.
  const auto legacy_map = node.declare_parameter<std::string>("middleware_config", "");
  if (id_map.empty())
  {
    id_map = legacy_map;
  }
  if (id_map.empty())
  {
    throw std::invalid_argument("Provide robot_id_map pointing to robot_id_map.yaml");
  }
  config.motors = MotorsDefinition::load(id_map, config.joints);
  config.lower =
      node.declare_parameter<std::vector<double>>("position_lower", std::vector<double>{});
  config.upper =
      node.declare_parameter<std::vector<double>>("position_upper", std::vector<double>{});
  config.velocity =
      node.declare_parameter<std::vector<double>>("max_velocity", std::vector<double>{});
  config.acceleration =
      node.declare_parameter<std::vector<double>>("max_acceleration", std::vector<double>{});

  const auto n = config.joints.size();
  if (config.lower.size() != n || config.upper.size() != n || config.velocity.size() != n ||
      config.acceleration.size() != n)
  {
    throw std::invalid_argument(
        "Provide position_lower/upper, max_velocity/acceleration for every joint");
  }

  for (size_t i = 0; i < n; ++i)
  {
    if (!std::isfinite(config.lower[i]) || !std::isfinite(config.upper[i]) ||
        config.lower[i] > config.upper[i] || !std::isfinite(config.velocity[i]) ||
        config.velocity[i] <= 0 || !std::isfinite(config.acceleration[i]) ||
        config.acceleration[i] <= 0)
    {
      throw std::invalid_argument("Invalid joint limits");
    }
  }

  const auto gains = node.declare_parameter<std::vector<double>>("gains", {0., 0., 0., 0., 0.});
  if (gains.size() != 5)
  {
    throw std::invalid_argument("Expected five gains");
  }

  for (size_t i = 0; i < 5; ++i)
  {
    if (!std::isfinite(gains[i]) || std::abs(gains[i]) > std::numeric_limits<float>::max())
    {
      throw std::invalid_argument("Invalid gain");
    }

    config.gains[i] = static_cast<float>(gains[i]);
  }

  return config;
}

size_t ControllerConfig::index(const std::string &name) const
{
  const auto it = std::find(joints.begin(), joints.end(), name);
  if (it == joints.end())
  {
    throw std::invalid_argument("Unknown joint: " + name);
  }

  return static_cast<size_t>(it - joints.begin());
}

std::vector<size_t> ControllerConfig::joint_indices(const std::vector<std::string> &names) const
{
  if (names.empty())
  {
    throw std::invalid_argument("Empty command joint names");
  }

  std::vector<size_t> indices;
  indices.reserve(names.size());
  for (const auto &name : names)
  {
    const auto i = index(name);
    if (std::find(indices.begin(), indices.end(), i) != indices.end())
    {
      throw std::invalid_argument("Duplicate command joint: " + name);
    }
    indices.push_back(i);
  }
  return indices;
}

void ControllerConfig::check(const trajectory_msgs::msg::JointTrajectoryPoint &point,
                             const std::vector<std::string> &names) const
{
  check_indexed(point, joint_indices(names));
}

void ControllerConfig::check_indexed(const trajectory_msgs::msg::JointTrajectoryPoint &point,
                                     const std::vector<size_t> &indices) const
{
  const auto count = indices.size();
  const bool has_velocity = !point.velocities.empty();
  const bool has_acceleration = !point.accelerations.empty();
  if (count == 0 || point.positions.size() != count ||
      (has_velocity && point.velocities.size() != count) ||
      (has_acceleration && point.accelerations.size() != count) || !point.effort.empty())
  {
    throw std::invalid_argument("Invalid command dimensions or effort");
  }

  for (size_t j = 0; j < count; ++j)
  {
    const auto i = indices[j];
    if (i >= joints.size())
    {
      throw std::invalid_argument("Invalid command joint index");
    }

    const double p = point.positions[j];
    const double v = has_velocity ? point.velocities[j] : 0;
    const double a = has_acceleration ? point.accelerations[j] : 0;
    if (!std::isfinite(p) || !std::isfinite(v) || !std::isfinite(a) || p < lower[i] ||
        p > upper[i] || std::abs(v) > velocity[i] || std::abs(a) > acceleration[i])
    {
      throw std::invalid_argument("Joint limit exceeded: " + joints[i]);
    }
  }
}

rclcpp::QoS declare_command_qos(rclcpp::Node &node)
{
  auto qos = rclcpp::QoS(1).reliable();
  const auto durability =
      node.declare_parameter<std::string>("command_durability", "transient_local");
  if (durability == "transient_local")
  {
    qos.transient_local();
  }
  else if (durability != "volatile")
  {
    throw std::invalid_argument("Invalid command_durability");
  }

  return qos;
}

trajectory_msgs::msg::JointTrajectoryPoint
joint_command(const sensor_msgs::msg::JointState &message)
{
  const std::set<std::string> names(message.name.begin(), message.name.end());
  if (message.name.empty() || names.count("") || names.size() != message.name.size() ||
      message.position.size() != message.name.size() ||
      (!message.velocity.empty() && message.velocity.size() != message.name.size()) ||
      !message.effort.empty())
  {
    throw std::invalid_argument(
        "Expected unique names, positions, optional velocities and no effort");
  }

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = message.position;
  point.velocities = message.velocity;
  return point;
}
} // namespace advrf_ros2_motors_controller
