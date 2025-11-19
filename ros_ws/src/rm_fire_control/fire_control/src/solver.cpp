#include "fire_control/solver.hpp"

//c++ system
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <memory>
#include <string>
#include <vector>
#include <cstdio>
//project



namespace rm_fire_control
{
Solver::Solver(std::weak_ptr<rclcpp::Node> n)  
: node_(n)
{
  auto node = node_.lock();

  //input parameter
  LargeArmorWidth_ = node->declare_parameter<double>("large_armor_width", 0.23);  
  SmallArmorWidth_ = node->declare_parameter<double>("small_armor_width", 0.135);
  LargeArmorHeight_ = node->declare_parameter<double>("large_armor_height", 0.127);
  SmallArmorHeight_ = node->declare_parameter<double>("small_armor_height", 0.125);
  ArmorPitch_ = (node->declare_parameter<double>("armor_pitch", 15.0) / 180.0 * M_PI); //装甲板pitch角
  SBias_ = node->declare_parameter<double>("s_bias", 0.0);  //odom到摩擦轮
  ZBias_ = node->declare_parameter<double>("z_bias", 0.0);
  K_ = node->declare_parameter<double>("k",0.025);  //弹丸参数（分小弹丸和大弹丸）
  Gravity_ = node->declare_parameter<double>("gravity", 9.79);

  YawMotorResSpeedA_ = node->declare_parameter<double>("yaw_motor_res_speed_a", 1.79e-3);  // 电机响应速度  ax + b
  YawMotorResSpeedB_ = node->declare_parameter<double>("yaw_motor_res_speed_b", 0.089);
  Transfer_Thresh_ = node->declare_parameter<int>("tranfer_thresh", 5);  //  跟踪状态转换阈值
  MaxTrackingVYaw1_ = node->declare_parameter<double>("max_tracking_v_yaw_1", 1.1);   //最大甲板跟踪角速度
  MaxTrackingVYaw2_ = node->declare_parameter<double>("max_tracking_v_yaw_2", 6.0);
  MaxOrientationAngle_ = (node->declare_parameter<double>("max_orientation_angle", 58.8) / 180.0 * M_PI);  //最大甲板跟踪角 角度转弧度
  MaxTrackingError_ = node->declare_parameter<double>("max_tracking_error", 0.8);  //1为可以打装甲板边缘
  MaxOutError_ = node->declare_parameter<double>("max_out_error", 0.6);  //装甲板位置出现的最大偏差

  
  CommuniDelay_ = node->declare_parameter<double>("communite_delay", 0.0);   //上下位机通信延迟
  ReceiveToFireDelay_ = node->declare_parameter<double>("receive_to_fire_delay", 0.01);  //接收信息到开火延迟

  Cur_V_ = node->declare_parameter<double>("current_v", 22.0);  //  m/s
  up_overflow_count_ = 0;
  down_overflow_count_ = 0;
  state_ = State::SLOW;
  

  node.reset();
}

fire_control_interfaces::msg::GimbalCmd Solver::Solve(const auto_aim_interfaces::msg::Target &target,
                                                      std::shared_ptr<tf2_ros::Buffer> tf2_buffer)
{   
  node_shared_ = node_.lock();
  
  // Get current yaw and pitch of gimbal
  try {
    auto gimbal_tf =
      tf2_buffer->lookupTransform(target.header.frame_id, "gimbal_link", tf2::TimePointZero);
    auto msg_q = gimbal_tf.transform.rotation;

    tf2::Quaternion tf_q;
    tf2::fromMsg(msg_q, tf_q);
    double roll;
    tf2::Matrix3x3(tf_q).getRPY(roll, cur_pitch_, cur_yaw_);
    cur_pitch_ = -cur_pitch_;
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(node_shared_->get_logger(), "armor_solver: %s", ex.what());
    throw ex;
  }


  armor_w_ = (target.id == std::string("1")) ? LargeArmorWidth_ : SmallArmorWidth_;
  armor_h_ = (target.id == std::string("1")) ? LargeArmorHeight_ : SmallArmorHeight_;

  if(std::abs(target.v_yaw) > MaxTrackingVYaw1_)
  {
    state_ = NORMAL;
    if(std::abs(target.v_yaw) > MaxTrackingVYaw2_)
    {
      state_ = FAST;
    }
  }
  else
  {
    state_ = SLOW;
  }

  double max_orientation_angle;

  // State change 转速慢就算上云台转动，转速快就直接打
  switch (state_) 
  {
    case SLOW: 
    {
      if (std::abs(target.v_yaw) > MaxTrackingVYaw1_) 
      {
        up_overflow_count_++;
      } else 
      {
        up_overflow_count_ = 0;
      }
      //state renew
      if (up_overflow_count_ > Transfer_Thresh_) 
      {
        state_ = NORMAL;
        up_overflow_count_ = 0;
        down_overflow_count_ = 0;
        max_orientation_angle = 0.0;
      }
      else
      {
        max_orientation_angle = MaxOrientationAngle_;
      }      
      break;
    }
    case NORMAL: 
    {
      if (std::abs(target.v_yaw) > MaxTrackingVYaw2_) 
      {
        up_overflow_count_++;
      } else 
      {
        up_overflow_count_ = 0;
      }
      if (std::abs(target.v_yaw) < MaxTrackingVYaw1_) 
      {
        down_overflow_count_++;
      } else 
      {
        down_overflow_count_ = 0;
      }
      //state renew
      if (up_overflow_count_ > Transfer_Thresh_) 
      {
        state_ = FAST;
        up_overflow_count_ = 0;
        max_orientation_angle = 0.0;
      }
      else if (down_overflow_count_ > Transfer_Thresh_) 
      {
        state_ = SLOW;
        up_overflow_count_ = 0;
        down_overflow_count_ = 0;
        max_orientation_angle = MaxOrientationAngle_;
      }
      else
      {
        max_orientation_angle = 0.0;
      }
      break;
    }
    case FAST:
    {
      if(std::abs(target.v_yaw) < MaxTrackingVYaw2_)
      {
        down_overflow_count_++;
      }else
      {
        down_overflow_count_ = 0;
      }
      //state renew
      if (down_overflow_count_ > Transfer_Thresh_) 
      {
        state_ = NORMAL;
        up_overflow_count_ = 0;
        down_overflow_count_ = 0;
        max_orientation_angle = 0.0;
      }
      else
      {
        max_orientation_angle = 0.0;
      }
      break;
    }
  }

  // Init
  fire_control_interfaces::msg::GimbalCmd gimbal_cmd;
  gimbal_cmd.header = target.header;
  gimbal_cmd.tracking = target.tracking;
  gimbal_cmd.id = target.id;
  gimbal_cmd.armors_num = target.armors_num;

  Pose chosen_aim_pose;
  //即云台跟随的信息
  HitInfo hit_aim_info;
  //云台跟随延迟信息
  Eigen::Vector3d target_position(target.position.x, target.position.y, target.position.z);
  double flying_time = GetFlyingTime(target_position);
  double dt = (node_shared_->get_clock()->now() - rclcpp::Time(target.header.stamp)).seconds() + CommuniDelay_ + flying_time;

  GetBestPose(target, dt, max_orientation_angle, chosen_aim_pose, hit_aim_info);
  
  if (hit_aim_info.distance == -1.0) 
  {  
    RCLCPP_ERROR(node_shared_->get_logger(), "Invalid distance detected.");
    gimbal_cmd.yaw = 0.0;
    gimbal_cmd.pitch = 0.0; 
    gimbal_cmd.distance = 0.0;

    gimbal_cmd.aim_x = 0.0;
    gimbal_cmd.aim_y = 0.0;
    gimbal_cmd.aim_z = 0.0;
  } 
  //弧度制
  gimbal_cmd.yaw = hit_aim_info.yaw;
  gimbal_cmd.pitch = hit_aim_info.pitch; 
  gimbal_cmd.distance = hit_aim_info.distance;

  gimbal_cmd.aim_x = chosen_aim_pose.position.x();
  gimbal_cmd.aim_y = chosen_aim_pose.position.y();
  gimbal_cmd.aim_z = chosen_aim_pose.position.z();
  //  change of angle
  gimbal_cmd.yaw_diff = hit_aim_info.yaw - cur_yaw_;
  gimbal_cmd.pitch_diff = - (hit_aim_info.pitch - cur_pitch_);
  // 加上开火延迟（拨弹延迟等）用于火控
  dt += ReceiveToFireDelay_;

  gimbal_cmd.fire_advice = FireCtrl(target, dt, max_orientation_angle, cur_yaw_, cur_pitch_, hit_aim_info, chosen_aim_pose);
  
  if (gimbal_cmd.fire_advice) {
    //RCLCPP_INFO(node_shared_->get_logger(), "You Need Fire!");
  }

  node_shared_.reset();
  return gimbal_cmd;
}

double Solver::GetFlyingTime(const Eigen::Vector3d &p)
{
  double distance = p.head(2).norm() + SBias_;
  double angle = std::atan2(p.z(), distance);
  double t = (double)((std::exp(K_ * distance) - 1) / (K_ * Cur_V_ * std::cos(angle)));
  return t;
}

std::vector<Pose> Solver::GetArmorPoses(const Eigen::Vector3d &target_center,
                                         const double target_yaw,
                                         const double r1,
                                         const double r2,
                                         const double dz,
                                         const size_t armors_num) 
{
  auto armors_poses = std::vector<Pose>(armors_num);
  // Calculate the position of each armor
  bool is_current_pair = true;
  for (size_t i = 0; i < armors_num; i++) 
  {
    if(armors_num == 4)
    {
      armors_poses[i].yaw = target_yaw + i * (2 * M_PI / armors_num);
      armors_poses[i].r = is_current_pair ? r1 : r2;
      armors_poses[i].dz =  is_current_pair ? 0 : dz;
      is_current_pair = !is_current_pair;
    }
    else
    {
      armors_poses[i].r = r1;
      armors_poses[i].dz = 0;
    }
    
    armors_poses[i].position =
      target_center + Eigen::Vector3d(-armors_poses[i].r * cos(armors_poses[i].yaw), -armors_poses[i].r * sin(armors_poses[i].yaw), armors_poses[i].dz);
  }
  return armors_poses;
}

//calculate the suitable gimbal pose
void Solver::CalcYawAndPitch(const Eigen::Vector3d &position, double &yaw, double &pitch)
{
  yaw = atan2(position.y(), position.x());
  pitch = PitchTrajectoryCompensation(position.head(2).norm() - SBias_, position.z() - ZBias_, Cur_V_);
}

double Solver::PitchTrajectoryCompensation(const double &s, const double &z, const double &v)
{
  double z_temp, z_actual, dz;
  double angle_pitch;
  int i = 0;
  z_temp = z;
  // iteration
  for (i = 0; i < 20; i++)
  {
    angle_pitch = atan2(z_temp, s); // rad
    z_actual = MonoDirectionalAirResistanceModel(s, v, angle_pitch);
    dz = 0.3*(z - z_actual);
    z_temp = z_temp + dz;
    // RCLCPP_INFO(node_shared_->get_logger(),   
    //             "iteration num %d: angle_pitch %f, temp target z:%f, err of z:%f, s:%f",   
    //             i + 1, angle_pitch * 180 / M_PI, z_temp, dz, s);  

    if (std::fabs(dz) < 0.00001)
    {
      break;
    }
  }
  return angle_pitch;
}

double Solver::MonoDirectionalAirResistanceModel(const double &s, const double &v, const double &angle)
{
  double t, z;
  //t为给定v与angle时的飞行时间
  t = (double)((std::exp(K_ * s) - 1) / (K_ * v * std::cos(angle)));
  //z为给定v与angle时的高度
  z = (double)(v * std::sin(angle) * t - Gravity_ * t * t / 2);
  // RCLCPP_INFO(node_shared_->get_logger(), "model %f %f\n", t, z);
  return z;
}

double Solver::AngleToGimbalX(const double &yaw, const double &cur_yaw)
{
  return std::atan2(std::sin(yaw - cur_yaw), std::cos(yaw - cur_yaw));
}

void Solver::GetBestPose(const auto_aim_interfaces::msg::Target &target,
                          const double &dt,
                          const double &max_orientation_angle,
                          Pose &chosen_pose,
                          HitInfo &hit_info)
{

  //  predict the the position of target
  Eigen::Vector3d target_position(target.position.x, target.position.y, target.position.z);
  Eigen::Vector3d target_velocity(target.velocity.x, target.velocity.y, target.velocity.z);
  double target_yaw = target.yaw;
  target_position += dt * target_velocity;
  target_yaw += dt * target.v_yaw;


  std::vector<Pose> armor_poses = GetArmorPoses(
      target_position, target_yaw, target.radius_1, target.radius_2, target.dz, target.armors_num);

  //select aim
  int best_armor_index = -1;
  
  double min_angle_to_x = 1000.0;
  for(int i = 0; i < target.armors_num; i++)
  {
    // Angle between armor and the X-axis
    double theta = AngleToGimbalX(armor_poses[i].yaw, std::atan2(target_position.y(), target_position.x()));

    if(std::abs(theta) <= max_orientation_angle)
    {
      if(std::abs(AngleToGimbalX(std::atan2(armor_poses[i].position.y(), armor_poses[i].position.x()), cur_yaw_)) < min_angle_to_x)
      {
        best_armor_index = i;
        min_angle_to_x = std::abs(AngleToGimbalX(std::atan2(armor_poses[i].position.y(), armor_poses[i].position.x()), cur_yaw_));
      }
    }

  }
  
  if(best_armor_index == -1)
  {
    // RCLCPP_INFO(node_shared_->get_logger(), "no drict armor");
    if(state_ != SLOW)
    {
      double min_armor_to_wait = 1000.0;
      int indirect_aim_armor_num = 0;

      for(int i = 0; i < target.armors_num; i++)
      {
        double max_out_angle = armor_w_ / 2.0 * MaxOutError_ / armor_poses[i].r;
        double theta = AngleToGimbalX(armor_poses[i].yaw, cur_yaw_);
        //装甲板到等待角
        double angle = (target.v_yaw > 0.0 ? -max_orientation_angle - theta : theta - max_orientation_angle) - M_PI + max_out_angle;
        double armor_to_wait = std::atan2(std::sin(angle), std::cos(angle)) + M_PI - max_out_angle;

        //选择最小角
        if(armor_to_wait  < min_armor_to_wait)
        {
          min_armor_to_wait = armor_to_wait;
          indirect_aim_armor_num = i;
        }
      }

      chosen_pose = armor_poses[indirect_aim_armor_num];
      //到等待角时间
      double time_to_wait_angle = min_armor_to_wait / std::abs(target.v_yaw);
      //预测位置
      target_position += time_to_wait_angle * target_velocity;
      target_yaw += time_to_wait_angle * target.v_yaw;
      
      hit_info.distance = target_position.norm();
      if (hit_info.distance < 0.1) {
        throw std::runtime_error("No valid armor to shoot");
      }
      std::vector<Pose> predict_armor_poses = GetArmorPoses(
        target_position, target_yaw, target.radius_1, target.radius_2, target.dz, target.armors_num);
      //瞄准
      CalcYawAndPitch(predict_armor_poses[indirect_aim_armor_num].position, hit_info.yaw, hit_info.pitch);
      //
    }
    else
    {
      hit_info.distance = -1.0;
      return;
    }
  }
  else
  {
    chosen_pose = armor_poses[best_armor_index];
    hit_info.distance = chosen_pose.position.norm();
    if (chosen_pose.position.norm() < 0.1) {
      throw std::runtime_error("No valid armor to shoot");
    }
    CalcYawAndPitch(armor_poses[best_armor_index].position, hit_info.yaw, hit_info.pitch);
  }
}      

/* ------------------------fire_control--------------------*/
bool Solver::FireCtrl(const auto_aim_interfaces::msg::Target &target,
                      const double &dt,
                      const double &max_orientation_angle,
                      const double &cur_yaw,
                      const double &cur_pitch,
                      const HitInfo &hit_aim_info,
                      const Pose &chosen_aim_pose)
{
  Eigen::Vector3d target_position(target.position.x, target.position.y, target.position.z);
  Eigen::Vector3d target_velocity(target.velocity.x, target.velocity.y, target.velocity.z);
  double target_yaw = target.yaw;
  target_position += dt * target_velocity;
  target_yaw += dt * target.v_yaw;

  std::vector<Pose> actual_armor_poses = GetArmorPoses(
      target_position, target_yaw, target.radius_1, target.radius_2, target.dz, target.armors_num);
  
  Pose chosen_actual_pose;
  HitInfo hit_actual_info;
 
  GetBestPose(target, dt, max_orientation_angle, chosen_actual_pose, hit_actual_info);
  
  //判断装甲板处于可击打范围内(之后改进方向，订阅detector，同步电控云台传输，可以改进装配误差)
  // RCLCPP_INFO(node_shared_->get_logger(), "kkkk%f", MaxTrackingError_);
  if(AimErrorExceeded(hit_aim_info, cur_yaw, cur_pitch, MaxTrackingError_))
  {
    //RCLCPP_INFO(node_shared_->get_logger(), "not true positon");
    return false;
  }

  if(state_ != SLOW)   //这里改逻辑，低速不触发
  {
    //云台指向打击状态与真实打击状态
    double yaw_aim_to_actual = chosen_actual_pose.yaw - chosen_aim_pose.yaw;
    yaw_aim_to_actual = std::atan2(std::sin(yaw_aim_to_actual), std::cos(yaw_aim_to_actual));
    //检测是否属于回转状态
    if(std::signbit(target.v_yaw) != std::signbit(yaw_aim_to_actual))
    {
      //计算装甲板回转角度
      double aim_theta = std::atan2(std::sin(chosen_aim_pose.yaw), std::cos(chosen_aim_pose.yaw));
      double armor_angle = target.v_yaw > 0.0 ? max_orientation_angle - aim_theta : -max_orientation_angle - aim_theta;
      double aim_hit_to_rotate_back = std::atan2(std::sin(armor_angle), std::cos(armor_angle));
      //各个时间
      double time_aim_hit = (node_shared_->get_clock()->now()).seconds() + CommuniDelay_ + GetFlyingTime(chosen_aim_pose.position);
      double time_actual_hit = (node_shared_->get_clock()->now()).seconds() + CommuniDelay_ + ReceiveToFireDelay_ + GetFlyingTime(chosen_actual_pose.position);
      double time_start_rotating_back = time_aim_hit + aim_hit_to_rotate_back / target.v_yaw;
      //回转时间后装甲板位置
      auto pos_when_start_rotating_back = PredictArmorPose(chosen_aim_pose, target_position - ReceiveToFireDelay_ * target_velocity, 
        target.yaw - ReceiveToFireDelay_ * target.v_yaw, target_velocity, target.v_yaw, aim_hit_to_rotate_back / target.v_yaw + ReceiveToFireDelay_, target.armors_num);
      HitInfo hit_when_start_rotating_back;
      CalcYawAndPitch(pos_when_start_rotating_back.position, hit_when_start_rotating_back.yaw, hit_when_start_rotating_back.pitch);
      //枪口回转角度
      double yaw_barrel_rotate_back = std::atan2(std::sin(hit_actual_info.yaw - hit_when_start_rotating_back.yaw),
       std::cos(hit_actual_info.yaw - hit_when_start_rotating_back.yaw));
      //枪口回转时间
      double time_end_rotating_back = (time_start_rotating_back + 
        YawMotorResSpeedA_ * std::abs(yaw_barrel_rotate_back) * 180.0 / M_PI + YawMotorResSpeedB_);

      if(time_start_rotating_back < time_actual_hit 
          && time_actual_hit < time_end_rotating_back)
      {
        //RCLCPP_INFO(node_shared_->get_logger(), "stay rotating");
        return false;
      }   
    }
  }

  double actual_yaw, actual_pitch;
  CalcYawAndPitch(chosen_actual_pose.position, actual_yaw, actual_pitch);
  if(AimErrorExceeded(hit_actual_info, actual_yaw, actual_pitch, MaxTrackingError_))
  {
    return false;
  }
  return true;

}


bool Solver::AimErrorExceeded(const HitInfo &hit_info, const double &cur_yaw, const double &cur_pitch, const double &error_rate)
{
  if(ErrorDiff(hit_info.distance, hit_info.yaw, cur_yaw)  > armor_w_ / 2.0 * 
   std::cos(std::atan2(std::sin(hit_info.yaw - cur_yaw), std::cos(hit_info.yaw - cur_yaw))) *
   error_rate)
  {
    return true;
  }

  // if(ErrorDiff(hit_info.distance, hit_info.pitch, cur_pitch) > armor_h_ / 2.0 * std::cos(ArmorPitch_ + ZBias_) * error_rate)
  // {
  //   return true;
  // }

  return false;
}

//误差距
double Solver::ErrorDiff(const double &distance, const double &aim_angle, const double &cur_angle)
{
  if (distance < 0.01) {  // 添加距离过小的保护
    return 1000.0;
  }

  double angle_diff = aim_angle - cur_angle;
  return std::fabs(2.0 * distance *
   std::sin(std::atan2(std::sin(angle_diff), std::cos(angle_diff)) / 2.0));
}

Pose Solver::PredictArmorPose(const Pose& current_armor_pose,
                      const Eigen::Vector3d& target_center_current,
                      double target_yaw_current,
                      const Eigen::Vector3d& velocity,
                      double v_yaw,
                      double time,
                      int armors_num)
{
  int index = (int)(std::round((current_armor_pose.yaw - target_yaw_current) / (2 * M_PI / armors_num)));
  if(index >= armors_num)
  {
    index = 0;
  }
  else if(index < 0)
  {
    index = armors_num - 1;
  }
  Pose predict_pose;
  predict_pose.r = current_armor_pose.r;
  predict_pose.dz = current_armor_pose.dz;
  // 计算新目标中心
  Eigen::Vector3d new_center = target_center_current + velocity * time;
  // 计算新目标yaw角
  double new_target_yaw = target_yaw_current + v_yaw * time;
    
  // 计算新装甲板yaw
  predict_pose.yaw = new_target_yaw + index * (2 * M_PI / armors_num);
    
  // 计算新位置
  predict_pose.position = new_center + 
    Eigen::Vector3d(-current_armor_pose.r * cos(predict_pose.yaw),
                      -current_armor_pose.r * sin(predict_pose.yaw),
                      current_armor_pose.dz);
    
  return predict_pose;
}

} //namespace rm_fire_control

// int main(int argc, char **argv)  
// {  
//     // 初始化 ROS 节点  
//     rclcpp::init(argc, argv);  
//     auto node = std::make_shared<rclcpp::Node>("test_node");  

//     // 创建 Solver 对象  
//     rm_fire_control::Solver solver(node);  

//     // 测试案例 1: 计算飞行时间  
//     Eigen::Vector3d target_position(10.0, 0.0, 0.0);  
//     double flying_time = solver.GetFlyingTime(target_position);  
//     std::cout << "Flying time: " << flying_time << " seconds" << std::endl;  

//     // 测试案例 2: 计算装甲板位置  
//     Eigen::Vector3d target_center(0.5, 0.0, 0.0);  
//     double target_yaw = 0.0;  
//     double r1 = 0.5, r2 = 0.3, dz = 0.1;  
//     size_t armors_num = 4;  
//     auto armor_poses = solver.GetArmorPoses(tar node_shared_->get_clock()->now(),:endl;  
//     }  

//     // 测试案例 3: 选择最佳装甲板  
//     int best_armor_index = solver.GetBestPose(armor_poses, target_center, target_yaw, 0.0, armors_num);  
//     std::cout << "Best armor index: " << best_armor_index << std::endl;  

//     // 关闭 ROS 节点  
//     rclcpp::shutdown();  
//     return 0;  
// }node