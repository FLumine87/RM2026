#include "shooter.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace auto_aim
{
Shooter::Shooter(const std::string & config_path) : last_command_{false, false, 0, 0}
{
  auto yaml = YAML::LoadFile(config_path);
  first_tolerance_ = tools::read<double>(yaml, "first_tolerance", 3.0) / 57.3;    // degree to rad
  second_tolerance_ = tools::read<double>(yaml, "second_tolerance", 1.5) / 57.3;  // degree to rad
  judge_distance_ = tools::read<double>(yaml, "judge_distance", 2.0);
  auto_fire_ = tools::read<bool>(yaml, "auto_fire", true);
  max_fire_yaw_angle_ = tools::read<double>(yaml, "max_fire_yaw_angle", 30.0) / 57.3;    // degree to rad
  fire_delay_ = tools::read<double>(yaml, "fire_delay", 0.01);
}

bool Shooter::shoot(
  const io::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos,
  double bullet_speed)
{
  if (!command.control || targets.empty() || !auto_fire_) return false;

  auto target_x = targets.front().ekf_x()[0];
  auto target_y = targets.front().ekf_x()[2];
  auto tolerance = std::sqrt(tools::square(target_x) + tools::square(target_y)) > judge_distance_
                     ? second_tolerance_
                     : first_tolerance_;
  // tools::logger()->debug("d(command.yaw) is {:.4f}", std::abs(last_command_.yaw - command.yaw));
  if (
    std::abs(last_command_.yaw - command.yaw) < tolerance * 2 &&  //此时认为command突变不应该射击
    std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance &&    //应该减去上一次command的yaw值
    aimer.debug_aim_point.valid) {
    auto target_ekf_x = targets.front().ekf_x();
    double armor_angle = target_ekf_x[6];
    double armor_yaw_vel = target_ekf_x[7];
    double target_x = target_ekf_x[0];
    double target_y = target_ekf_x[2];
    double distance = std::sqrt(tools::square(target_x) + tools::square(target_y));
    double fly_time = distance / bullet_speed + fire_delay_;
    double future_armor_angle = tools::limit_rad(armor_angle + armor_yaw_vel * fly_time);
    if (future_armor_angle < max_fire_yaw_angle_) {
      last_command_ = command;
      return true;
    }
  }

  last_command_ = command;
  return false;
}

}  // namespace auto_aim