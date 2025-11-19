

#ifndef FIRE_CONTROL__SOLVER_HPP_
#define FIRE_CONTROL__SOLVER_HPP_

// Eigen

#include <eigen3/Eigen/Dense>
//ros2
#include <rclcpp/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

// project
#include "fire_control_interfaces/msg/gimbal_cmd.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

//c++ system
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <memory>
#include <string>
#include <vector>
#include <cstdio>

namespace rm_fire_control
{
  
struct Pose
{
  Eigen::Vector3d position;
  double yaw;
  double r;
  double dz;
};

struct HitInfo
{
  double distance;
  double yaw;
  double pitch;
  double distance_v;
  double yaw_v;
  double pitch_v;
};

class Solver
{
public:
  Solver(std::weak_ptr<rclcpp::Node> node); 

  fire_control_interfaces::msg::GimbalCmd Solve(const auto_aim_interfaces::msg::Target &target_msg,
                                      std::shared_ptr<tf2_ros::Buffer> tf2_buffer);

  enum State { SLOW = 0, NORMAL = 1, FAST = 2 } state_;
private:
  double GetFlyingTime(const Eigen::Vector3d &p);

  std::vector<Pose> GetArmorPoses(const Eigen::Vector3d &target_center,
                                  const double target_yaw,
                                  const double r1,
                                  const double r2,
                                  const double dz,
                                  const size_t armors_num);


  void CalcYawAndPitch(const Eigen::Vector3d &position, double &yaw, double &pitch);

  double PitchTrajectoryCompensation(const double &s, const double &z, const double &v);

  double MonoDirectionalAirResistanceModel(const double &s, const double &v, const double &angle);

  double AngleToGimbalX(const double &yaw, const double &cur_yaw);

  void ChangeOfAngle(const HitInfo &Hit_info,
                      const double &Cur_yaw_,
                      const double &Cur_pitch_,
                      const double &Cur_roll_,
                      double &gimbal_cmd_yaw_diff
                      double &gimbal_cmd_pitch_diff);
  
  void GetBestPose(const auto_aim_interfaces::msg::Target &target,
                          const double &dt,
                          const double &max_orientation_angle,
                          Pose &chosen_pose,
                          HitInfo &hit_info);

  bool FireCtrl(const auto_aim_interfaces::msg::Target &target,
                      const double &dt,
                      const double &max_orientation_angle,
                      const double &cur_yaw,
                      const double &cur_pitch,
                      const HitInfo &hit_aim_info,
                      const Pose &chosen_aim_pose);

  bool AimErrorExceeded(const HitInfo &hit_info, const double &cur_yaw, const double &cur_pitch, const double &error_rate);

  double ErrorDiff(const double &distance, const double &aim_angle, const double &cur_angle);
  
  Pose PredictArmorPose(const Pose& current_armor_pose,
                      const Eigen::Vector3d& target_center_current,
                      double target_yaw_current,
                      const Eigen::Vector3d& velocity,
                      double v_yaw,
                      double time,
                      int armors_num);
  
  double LargeArmorWidth_;
  double SmallArmorWidth_;
  double LargeArmorHeight_;
  double SmallArmorHeight_;
  double ArmorPitch_;
  double SBias_;
  double ZBias_;
  double K_;
  double Gravity_; 
  
  double YawMotorResSpeedA_;
  double YawMotorResSpeedB_;
  double Transfer_Thresh_;
  double MaxTrackingVYaw1_;
  double MaxTrackingVYaw2_;
  double MaxOrientationAngle_;

  double MaxTrackingError_;
  double MaxOutError_;

  
  double CommuniDelay_;
  double ReceiveToFireDelay_;

  double Cur_V_;
  int up_overflow_count_;
  int down_overflow_count_;
  double cur_roll_;//在这里增加了roll角
  double cur_yaw_;
  double cur_pitch_;
  double armor_w_;
  double armor_h_;

  std::weak_ptr<rclcpp::Node> node_; 
  //用与链接node的shared_ptr
  std::shared_ptr<rclcpp::Node> node_shared_;   
};


  
}
#endif