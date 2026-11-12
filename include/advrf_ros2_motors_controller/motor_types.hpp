#pragma once

#include <cstdint>
#include <array>
#include <map>
#include <string>
#include <vector>

namespace advrf_ros2_motors_controller
{
class MotorsDefinition
{
public:
  void declare(int ecat_id, const std::string &joint_name);
  void set_gains(const std::string &joint, const std::array<float, 5> &gains);
  const std::array<float, 5> &gains_for_joint(
      const std::string &joint, const std::array<float, 5> &fallback) const;
  int32_t get_ecat_id(const std::string &joint_name) const;
  static MotorsDefinition from_ids(const std::vector<int64_t> &ids,
                                   const std::vector<std::string> &joints);

private:
  std::map<std::string, std::array<float, 5>> joint_gains_;
  std::map<std::string, int32_t> joint_name_to_ecat_id_;
};
} // namespace advrf_ros2_motors_controller
