#include "planner.hpp"

#include <vector>
#include <cmath>

#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace auto_aim
{

// ==================== 五次多项式实现 ====================
void QuinticPolynomial::solve(
    double start_pos, double start_vel, double start_acc,
    double end_pos, double end_vel, double end_acc,
    double T) {
    
    Eigen::MatrixXd A(6, 6);
    Eigen::VectorXd b(6);
    
    A.row(0) << 1, 0, 0, 0, 0, 0;
    b(0) = start_pos;
    
    A.row(1) << 0, 1, 0, 0, 0, 0;
    b(1) = start_vel;
    
    A.row(2) << 0, 0, 2, 0, 0, 0;
    b(2) = start_acc;
    
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;
    
    A.row(3) << 1, T, T2, T3, T4, T5;
    b(3) = end_pos;
    
    A.row(4) << 0, 1, 2*T, 3*T2, 4*T3, 5*T4;
    b(4) = end_vel;
    
    A.row(5) << 0, 0, 2, 6*T, 12*T2, 20*T3;
    b(5) = end_acc;
    
    coeffs_ = A.colPivHouseholderQr().solve(b);
}

double QuinticPolynomial::pos(double t) const {
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;
    return coeffs_(0) + coeffs_(1)*t + coeffs_(2)*t2 + coeffs_(3)*t3 + coeffs_(4)*t4 + coeffs_(5)*t5;
}

double QuinticPolynomial::vel(double t) const {
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    return coeffs_(1) + 2*coeffs_(2)*t + 3*coeffs_(3)*t2 + 4*coeffs_(4)*t3 + 5*coeffs_(5)*t4;
}

double QuinticPolynomial::acc(double t) const {
    double t2 = t * t;
    double t3 = t2 * t;
    return 2*coeffs_(2) + 6*coeffs_(3)*t + 12*coeffs_(4)*t2 + 20*coeffs_(5)*t3;
}

double QuinticPolynomial::max_acc(double T) const {
    double max_acc = 0;
    for (double t = 0; t <= T; t += 0.001) {
        max_acc = std::max(max_acc, std::abs(acc(t)));
    }
    return max_acc;
}

// ==================== Planner 实现 ====================
Planner::Planner(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  
  yaw_offset_ = tools::read<double>(yaml, "yaw_offset") / 57.3;
  pitch_offset_ = tools::read<double>(yaml, "pitch_offset") / 57.3;
  fire_thresh_ = tools::read<double>(yaml, "fire_thresh");
  convergence_thresh_ = tools::read<double>(yaml, "convergence_thresh");
  decision_speed_ = tools::read<double>(yaml, "decision_speed");
  high_speed_delay_time_ = tools::read<double>(yaml, "high_speed_delay_time");
  low_speed_delay_time_ = tools::read<double>(yaml, "low_speed_delay_time");
  fire_delay_ = tools::read<double>(yaml, "fire_delay");
  mid_ratio_ = tools::read<double>(yaml, "mid_ratio");
  
  // 显式方案参数
  transition_ratio_ = tools::read<double>(yaml, "transition_ratio", 0.3);
  
  // 动力学约束
  max_yaw_acc_ = tools::read<double>(yaml, "max_yaw_acc");
  max_pitch_acc_ = tools::read<double>(yaml, "max_pitch_acc");
  
  skip_convergence_check_ = false;
}

Plan Planner::plan(Target target, double bullet_speed, double current_yaw, double current_pitch)
{
  // 1. Predict fly_time
  Eigen::Vector3d xyz;
  auto min_dist = 1e10;
  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
    }
  }
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  target.predict(bullet_traj.fly_time);

  // 2. Get trajectory
  int dummy_idx;
  yaw0_ = aim(target, bullet_speed, dummy_idx)(0);
  shoot_traj_ = get_trajectory(target, yaw0_, bullet_speed);

  // 3. 预计算切换点信息
  precompute_switch_info(target);

  // 4. 计算关键时间点的状态
  int fire_idx = static_cast<int>(fire_delay_ / DT);
  int gimbal_idx = HALF_HORIZON;

  // 原点状态（用于收敛检测）
  State origin_yaw_state = get_state_at(0, shoot_traj_);
  State origin_pitch_state = {shoot_traj_(2, 0), shoot_traj_(3, 0), 0};

  // fire_delay时刻状态（用于开火判断）
  State fire_yaw_state = get_state_at(fire_idx, shoot_traj_);
  State fire_pitch_state = get_state_at(fire_idx, shoot_traj_);

  // gimbal_delay时刻状态（用于发送指令）
  State gimbal_yaw_state = get_state_at(gimbal_idx, shoot_traj_);
  State gimbal_pitch_state = get_state_at(gimbal_idx, shoot_traj_);

  // 5. 填充plan
  Plan plan;
  plan.control = true;

  plan.target_yaw = tools::limit_rad(shoot_traj_(0, gimbal_idx) + yaw0_);
  plan.target_pitch = shoot_traj_(2, gimbal_idx);

  plan.yaw = tools::limit_rad(gimbal_yaw_state.pos + yaw0_);
  plan.yaw_vel = gimbal_yaw_state.vel;
  plan.yaw_acc = gimbal_yaw_state.acc;

  plan.pitch = gimbal_pitch_state.pos;
  plan.pitch_vel = gimbal_pitch_state.vel;
  plan.pitch_acc = gimbal_pitch_state.acc;

  // 6. 开火决策
  bool trajectory_error = std::hypot(
      shoot_traj_(0, fire_idx) - fire_yaw_state.pos,
      shoot_traj_(2, fire_idx) - fire_pitch_state.pos) < fire_thresh_;
  
  bool gimbal_converged = skip_convergence_check_ || (
      std::hypot(
          current_yaw - origin_yaw_state.pos,
          current_pitch - origin_pitch_state.pos) < convergence_thresh_);
  
  bool in_valid_fire_region = is_in_valid_fire_region(fire_idx);
  
  plan.fire = trajectory_error && gimbal_converged && in_valid_fire_region;
  
  return plan;
}

Plan Planner::plan(std::optional<Target> target, double bullet_speed)
{
  if (!target.has_value()) return {false};
  skip_convergence_check_ = true;
  return plan(*target, bullet_speed, 0.0, 0.0);
}

Plan Planner::plan(std::optional<Target> target, double bullet_speed, double current_yaw, double current_pitch)
{
  if (!target.has_value()) return {false};

  double delay_time =
    std::abs(target->ekf_x()[7]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

  auto future = std::chrono::steady_clock::now() + std::chrono::microseconds(int(delay_time * 1e6));

  target->predict(future);

  return plan(*target, bullet_speed, current_yaw, current_pitch);
}

Plan Planner::plan(Target target, double bullet_speed)
{
  skip_convergence_check_ = true;
  return plan(target, bullet_speed, 0.0, 0.0);
}

void Planner::precompute_switch_info(Target& target) {
    switch_info_list_.clear();
    
    double omega = std::abs(target.ekf_x()[7]);
    
    for (int i = 0; i < HORIZON; i++) {
        if (switch_points_[i]) {
            SwitchInfo info;
            info.idx = i;
            
            // 计算最小过渡时间（基于动力学约束）
            double delta_vel = omega;  // 角速度变化量
            double min_T = delta_vel / max_yaw_acc_;
            
            // 根据 transition_ratio 调整过渡时间
            double T = min_T * (1 + transition_ratio_);
            
            // 计算过渡段起点和终点（对称设计）
            int half_steps = static_cast<int>(T / 2 / DT);
            info.start_idx = std::max(0, i - half_steps);
            info.end_idx = std::min(HORIZON - 1, i + half_steps);
            info.T = (info.end_idx - info.start_idx) * DT;
            
            // 获取边界状态
            double start_yaw = shoot_traj_(0, info.start_idx);
            double start_yaw_vel = shoot_traj_(1, info.start_idx);
            double end_yaw = shoot_traj_(0, info.end_idx);
            double end_yaw_vel = shoot_traj_(1, info.end_idx);
            
            double start_pitch = shoot_traj_(2, info.start_idx);
            double start_pitch_vel = shoot_traj_(3, info.start_idx);
            double end_pitch = shoot_traj_(2, info.end_idx);
            double end_pitch_vel = shoot_traj_(3, info.end_idx);
            
            // 求解五次多项式
            info.yaw_poly.solve(start_yaw, start_yaw_vel, 0, end_yaw, end_yaw_vel, 0, info.T);
            info.pitch_poly.solve(start_pitch, start_pitch_vel, 0, end_pitch, end_pitch_vel, 0, info.T);
            
            // 检查并调整过渡时间（满足动力学约束）
            double max_yaw_acc = info.yaw_poly.max_acc(info.T);
            while (max_yaw_acc > max_yaw_acc_ && info.end_idx < HORIZON - 1) {
                info.end_idx++;
                info.T = (info.end_idx - info.start_idx) * DT;
                end_yaw = shoot_traj_(0, info.end_idx);
                end_yaw_vel = shoot_traj_(1, info.end_idx);
                info.yaw_poly.solve(start_yaw, start_yaw_vel, 0, end_yaw, end_yaw_vel, 0, info.T);
                max_yaw_acc = info.yaw_poly.max_acc(info.T);
            }
            
            switch_info_list_.push_back(info);
        }
    }
}

State Planner::get_state_at(int idx, const Trajectory& traj) {
    // 检查是否在过渡段内
    for (const auto& info : switch_info_list_) {
        if (idx >= info.start_idx && idx <= info.end_idx) {
            double t_local = (idx - info.start_idx) * DT;
            return {
                info.yaw_poly.pos(t_local),
                info.yaw_poly.vel(t_local),
                info.yaw_poly.acc(t_local)
            };
        }
    }
    
    // 不在过渡段，直接返回原始轨迹
    double acc = 0;
    if (idx > 0 && idx < HORIZON - 1) {
        acc = (traj(1, idx) - traj(1, idx - 1)) / DT;
    }
    
    return {traj(0, idx), traj(1, idx), acc};
}

bool Planner::is_in_valid_fire_region(int shoot_offset) {
    int current_follow_start = 0;
    int current_follow_end = HORIZON - 1;
    bool found_switch_points = false;
    
    for (int i = shoot_offset - 1; i >= 0; i--) {
        if (switch_points_[i]) {
            current_follow_start = i + 1;
            found_switch_points = true;
            break;
        }
    }
    
    for (int i = shoot_offset + 1; i < HORIZON; i++) {
        if (switch_points_[i]) {
            current_follow_end = i - 1;
            found_switch_points = true;
            break;
        }
    }
    
    if (found_switch_points) {
        int follow_length = current_follow_end - current_follow_start;
        int mid_start = current_follow_start + follow_length * (1 - mid_ratio_) / 2;
        int mid_end = current_follow_end - follow_length * (1 - mid_ratio_) / 2;
        return (shoot_offset >= mid_start && shoot_offset <= mid_end);
    }
    
    return true;
}

Eigen::Matrix<double, 2, 1> Planner::aim(const Target & target, double bullet_speed, int & selected_armor_idx)
{
  Eigen::Vector3d xyz;
  double yaw;
  auto min_dist = 1e10;
  selected_armor_idx = 0;

  for (int i = 0; i < target.armor_xyza_list().size(); i++) {
    const auto & xyza = target.armor_xyza_list()[i];
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
      yaw = xyza[3];
      selected_armor_idx = i;
    }
  }
  debug_xyza = Eigen::Vector4d(xyz.x(), xyz.y(), xyz.z(), yaw);

  auto azim = std::atan2(xyz.y(), xyz.x());
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  if (bullet_traj.unsolvable) throw std::runtime_error("Unsolvable bullet trajectory!");

  return {tools::limit_rad(azim + yaw_offset_), -bullet_traj.pitch - pitch_offset_};
}

Trajectory Planner::get_trajectory(Target & target, double yaw0, double bullet_speed)
{
  Trajectory traj;

  target.predict(-DT * (HALF_HORIZON + 1));
  int last_selected_armor = 0;
  auto yaw_pitch_last = aim(target, bullet_speed, last_selected_armor);

  target.predict(DT);
  int current_selected_armor = 0;
  auto yaw_pitch = aim(target, bullet_speed, current_selected_armor);
  
  switch_points_.resize(HORIZON, 0);

  for (int i = 0; i < HORIZON; i++) {
    target.predict(DT);
    int next_selected_armor = 0;
    auto yaw_pitch_next = aim(target, bullet_speed, next_selected_armor);

    double yaw_diff = yaw_pitch_next(0) - yaw_pitch_last(0);
    if (yaw_diff > CV_PI) yaw_diff -= 2 * CV_PI;
    if (yaw_diff < -CV_PI) yaw_diff += 2 * CV_PI;
    auto yaw_vel = yaw_diff / (2 * DT);
    
    auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

    double relative_yaw = yaw_pitch(0) - yaw0;
    if (relative_yaw > CV_PI) relative_yaw -= 2 * CV_PI;
    if (relative_yaw < -CV_PI) relative_yaw += 2 * CV_PI;
    
    traj.col(i) << relative_yaw, yaw_vel, yaw_pitch(1), pitch_vel;

    if (next_selected_armor != current_selected_armor) {
      switch_points_[i] = 1;
    }
    
    last_selected_armor = current_selected_armor;
    current_selected_armor = next_selected_armor;

    yaw_pitch_last = yaw_pitch;
    yaw_pitch = yaw_pitch_next;
  }

  return traj;
}

PlanDebug Planner::debug(Target target, double bullet_speed)
{
  PlanDebug debug_result;
  debug_result.control = true;

  Eigen::Vector3d xyz;
  auto min_dist = 1e10;
  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
    }
  }
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  target.predict(bullet_traj.fly_time);

  int dummy_idx;
  yaw0_ = aim(target, bullet_speed, dummy_idx)(0);
  shoot_traj_ = get_trajectory(target, yaw0_, bullet_speed);
  precompute_switch_info(target);

  for (int i = 0; i < HORIZON; i++) {
    State yaw_state = get_state_at(i, shoot_traj_);
    State pitch_state = get_state_at(i, shoot_traj_);
    
    double plan_yaw = tools::limit_rad(yaw_state.pos + yaw0_);
    double plan_pitch = pitch_state.pos;
    double target_yaw = tools::limit_rad(shoot_traj_(0, i) + yaw0_);
    double target_pitch = shoot_traj_(2, i);

    debug_result.target_yaw_list.push_back(static_cast<float>(target_yaw));
    debug_result.target_pitch_list.push_back(static_cast<float>(target_pitch));
    debug_result.plan_yaw_list.push_back(static_cast<float>(plan_yaw));
    debug_result.plan_pitch_list.push_back(static_cast<float>(plan_pitch));

    bool trajectory_error = std::hypot(
        shoot_traj_(0, i) - yaw_state.pos,
        shoot_traj_(2, i) - pitch_state.pos) < fire_thresh_;

    bool in_valid_fire_region = is_in_valid_fire_region(i);

    debug_result.fireable_list.push_back(trajectory_error && in_valid_fire_region);
  }

  return debug_result;
}

}  // namespace auto_aim