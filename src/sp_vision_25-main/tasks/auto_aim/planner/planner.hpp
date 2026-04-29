#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <list>
#include <optional>
#include <vector>

#include "tasks/auto_aim/target.hpp"

namespace auto_aim
{
constexpr double DT = 0.01;
constexpr int HALF_HORIZON = 50;
constexpr int HORIZON = HALF_HORIZON * 2;

using Trajectory = Eigen::Matrix<double, 4, HORIZON>;  // yaw, yaw_vel, pitch, pitch_vel

struct Plan
{
  bool control;
  bool fire;
  float target_yaw;
  float target_pitch;
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
};

struct PlanDebug
{
  bool control;
  std::vector<float> target_yaw_list;
  std::vector<float> target_pitch_list;
  std::vector<float> plan_yaw_list;
  std::vector<float> plan_pitch_list;
  std::vector<bool> fireable_list;
};

// 状态结构体
struct State {
    double pos;   // 位置
    double vel;   // 速度
    double acc;   // 加速度
};

// 五次多项式类
class QuinticPolynomial {
public:
    Eigen::VectorXd coeffs_;  // 6个系数
    
    QuinticPolynomial() : coeffs_(6) {}
    
    // 根据边界条件求解
    void solve(
        double start_pos, double start_vel, double start_acc,
        double end_pos, double end_vel, double end_acc,
        double T);
    
    // 计算t时刻的位置
    double pos(double t) const;
    
    // 计算t时刻的速度
    double vel(double t) const;
    
    // 计算t时刻的加速度
    double acc(double t) const;
    
    // 计算最大加速度
    double max_acc(double T) const;
};

// 切换点信息结构体
struct SwitchInfo {
    int idx;                    // 切换点位置
    int start_idx;              // 过渡段起点
    int end_idx;                // 过渡段终点
    double T;                   // 过渡时间
    QuinticPolynomial yaw_poly; // yaw轴多项式
    QuinticPolynomial pitch_poly; // pitch轴多项式
};

class Planner
{
public:
  Eigen::Vector4d debug_xyza;
  Planner(const std::string & config_path);

  Plan plan(Target target, double bullet_speed);
  Plan plan(Target target, double bullet_speed, double current_yaw, double current_pitch);
  Plan plan(std::optional<Target> target, double bullet_speed);
  Plan plan(std::optional<Target> target, double bullet_speed, double current_yaw, double current_pitch);
  
  PlanDebug debug(Target target, double bullet_speed);

private:
  // 参数
  double yaw_offset_;
  double pitch_offset_;
  double fire_thresh_;
  double convergence_thresh_;
  double decision_speed_;
  double high_speed_delay_time_;
  double low_speed_delay_time_;
  double fire_delay_;
  double mid_ratio_;
  
  // 显式方案参数
  double transition_ratio_;     // 过渡段比例
  
  // 动力学约束
  double max_yaw_acc_;
  double max_pitch_acc_;
  
  bool skip_convergence_check_;
  std::vector<int> switch_points_;
  std::vector<SwitchInfo> switch_info_list_;
  
  // 轨迹信息（用于预计算）
  Trajectory shoot_traj_;
  double yaw0_;

  // 核心方法
  Eigen::Matrix<double, 2, 1> aim(const Target & target, double bullet_speed, int & selected_armor_idx);
  Trajectory get_trajectory(Target & target, double yaw0, double bullet_speed);
  
  // 显式方案核心方法
  void precompute_switch_info(Target& target);
  State get_state_at(int idx, const Trajectory& traj);
  bool is_in_valid_fire_region(int shoot_offset);
  
  // 辅助方法
  double calculate_min_transition_time(double delta_yaw, double delta_vel);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PLANNER_HPP