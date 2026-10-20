#include "advrf_ros2_motors_controller/motor_types.hpp"
#include <yaml-cpp/yaml.h>
#include <limits>
#include <set>
#include <stdexcept>

namespace advrf_ros2_motors_controller
{
void MotorsDefinition::declare(int ecat_id, const std::string &joint_name)
{
  if (ecat_id < 0 || joint_name.empty() || joint_name_to_ecat_id_.count(joint_name))
  {
    throw std::invalid_argument("Invalid or duplicate motor declaration: " + joint_name);
  }

  for (const auto &item : joint_name_to_ecat_id_)
  {
    if (item.second == ecat_id)
    {
      throw std::invalid_argument("Duplicate EtherCAT ID");
    }
  }

  joint_name_to_ecat_id_.emplace(joint_name, ecat_id);
}

int32_t MotorsDefinition::get_ecat_id(const std::string &joint_name) const
{
  const auto it = joint_name_to_ecat_id_.find(joint_name);
  if (it == joint_name_to_ecat_id_.end())
  {
    throw std::invalid_argument("Joint absent from joint_map: " + joint_name);
  }

  return it->second;
}

MotorsDefinition MotorsDefinition::load(const std::string &path,
                                        const std::vector<std::string> &joints)
{
  const auto map = YAML::LoadFile(path)["joint_map"];
  if (!map.IsMap())
  {
    throw std::invalid_argument("Expected joint_map mapping in robot_id_map file");
  }
  MotorsDefinition all, selected;
  for (const auto &entry : map)
  {
    const auto id = entry.first.as<int64_t>();
    if (id < 0 || id > std::numeric_limits<int32_t>::max())
    {
      throw std::invalid_argument("EtherCAT ID outside int32 range");
    }

    all.declare(static_cast<int>(id), entry.second.as<std::string>());
  }

  if (joints.empty())
  {
    throw std::invalid_argument("Explicit controlled joints required");
  }

  for (const auto &joint : joints)
  {
    selected.declare(all.get_ecat_id(joint), joint);
  }

  return selected;
}
} // namespace advrf_ros2_motors_controller
