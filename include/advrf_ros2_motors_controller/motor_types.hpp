#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace advrf_ros2_motors_controller
{
class MotorsDefinition
{
public:
  void declare(int ecat_id, const std::string &joint_name);
  int32_t get_ecat_id(const std::string &joint_name) const;
  // Reads robot_id_map joint_map: {ecat_id: joint_name}. Only selected joints are motors.
  static MotorsDefinition load(const std::string &path, const std::vector<std::string> &joints);

private:
  std::map<std::string, int32_t> joint_name_to_ecat_id_;
};
} // namespace advrf_ros2_motors_controller
