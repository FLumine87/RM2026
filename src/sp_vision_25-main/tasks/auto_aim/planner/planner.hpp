#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <list>
#include <optional>
#include <vector>

#include "tasks/auto_aim/target.hpp"

namespace auto_aim
{
constexpr double DT = 0.001;  // 1ms 步长
constexpr int TIME_MIN = -1;   // 时间轴起点（防止越界）
constexpr int TIME_MAX = 501;  // 时间轴终点
constexpr int TIME_RANGE = TIME_MAX - TIME_MIN + 1;

struct State {
    double yaw;
    double yaw_vel;
    double yaw_acc;
    double pitch;
    double pitch_vel;
    double pitch_acc;
};

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

// 五次多项式类
class QuinticPolynomial {
public:
    Eigen::VectorXd coeffs_;  // 6个系数
    
  QuinticPolynomial() : coeffs_(Eigen::VectorXd::Zero(6)) {}
    
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

// 切换点事件结构
struct SwitchEvent {
    int t_switch;          // 切换点时间（单位：DT步）
    int t_start;           // 过渡段开始时间
    int t_end;             // 过渡段结束时间
    double yaw_start;      // 过渡段起点yaw
    double yaw_vel_start;  // 过渡段起点角速度
    double yaw_end;        // 过渡段终点yaw
    double yaw_vel_end;    // 过渡段终点角速度
    double pitch_start;    // 过渡段起点pitch
    double pitch_vel_start;// 过渡段起点pitch角速度
    double pitch_end;      // 过渡段终点pitch
    double pitch_vel_end;  // 过渡段终点pitch角速度
    QuinticPolynomial yaw_poly;  // yaw轴多项式
    QuinticPolynomial pitch_poly; // pitch轴多项式
};

class Planner
{
public:
  Eigen::Vector4d debug_xyza{Eigen::Vector4d::Zero()};
  Planner(const std::string & config_path);

  Plan plan(Target target, double bullet_speed, double current_yaw, double current_pitch);
  Plan plan(Target target, double bullet_speed);
  Plan plan(std::optional<Target> target, double bullet_speed);
  Plan plan(std::optional<Target> target, double bullet_speed, double current_yaw, double current_pitch);

  PlanDebug debug(Target target, double bullet_speed);

private:
  // 配置参数（带默认值）
  double yaw_offset_{0.0};
  double pitch_offset_{0.0};
  double fire_thresh_{0.02};
  double convergence_thresh_{0.1};
  double decision_speed_{8.0};
  double high_speed_delay_time_{0.025};
  double low_speed_delay_time_{0.015};
  double fire_delay_{0.01};
  double mid_ratio_{0.4};
  double transition_ratio_{0.3};
  double max_yaw_acc_{50.0};
  double max_pitch_acc_{100.0};
  
  bool skip_convergence_check_{false};
  std::vector<SwitchEvent> switch_events_;

  // 核心方法
  Eigen::Matrix<double, 2, 1> aim(
    const Target & target, double bullet_speed, int & selected_armor_idx, bool update_debug_xyza = false);
  void precompute_switch_events(Target& target, double bullet_speed, double yaw0, double fly_time, double gimbal_delay);
  State get_state_at_time(Target& target, double bullet_speed, double yaw0, double time, double fly_time);
  State get_follow_state(Target& target, double bullet_speed, double yaw0, int t, double fly_time);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PLANNER_HPP