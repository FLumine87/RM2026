#include "planner.hpp"

#include <vector>

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
    
    // s(0) = start_pos
    A.row(0) << 1, 0, 0, 0, 0, 0;
    b(0) = start_pos;
    
    // s'(0) = start_vel
    A.row(1) << 0, 1, 0, 0, 0, 0;
    b(1) = start_vel;
    
    // s''(0) = start_acc
    A.row(2) << 0, 0, 2, 0, 0, 0;
    b(2) = start_acc;
    
    // s(T) = end_pos
    A.row(3) << 1, T, T*T, T*T*T, T*T*T*T, T*T*T*T*T;
    b(3) = end_pos;
    
    // s'(T) = end_vel
    A.row(4) << 0, 1, 2*T, 3*T*T, 4*T*T*T, 5*T*T*T*T;
    b(4) = end_vel;
    
    // s''(T) = end_acc
    A.row(5) << 0, 0, 2, 6*T, 12*T*T, 20*T*T*T;
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
    for (double t = 0; t <= T; t += 0.0001) {
        max_acc = std::max(max_acc, std::abs(acc(t)));
    }
    return max_acc;
}

// ==================== Planner 实现 ====================
Planner::Planner(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  
  // 读取配置参数（带默认值）
  yaw_offset_ = tools::read<double>(yaml, "yaw_offset", 0.0) / 57.3;
  pitch_offset_ = tools::read<double>(yaml, "pitch_offset", 0.0) / 57.3;
  fire_thresh_ = tools::read<double>(yaml, "fire_thresh", 0.02);
  convergence_thresh_ = tools::read<double>(yaml, "convergence_thresh", 0.1);
  decision_speed_ = tools::read<double>(yaml, "decision_speed", 8.0);
  high_speed_delay_time_ = tools::read<double>(yaml, "high_speed_delay_time", 0.025);
  low_speed_delay_time_ = tools::read<double>(yaml, "low_speed_delay_time", 0.015);
  fire_delay_ = tools::read<double>(yaml, "fire_delay", 0.01);
  mid_ratio_ = tools::read<double>(yaml, "mid_ratio", 0.4);
  transition_ratio_ = tools::read<double>(yaml, "transition_ratio", 0.3);
  max_yaw_acc_ = tools::read<double>(yaml, "max_yaw_acc", 50.0);
  max_pitch_acc_ = tools::read<double>(yaml, "max_pitch_acc", 100.0);
}

Plan Planner::plan(Target target, double bullet_speed, double current_yaw, double current_pitch)
{
  // 1. 计算弹丸飞行时间（不提前预测，目标状态在检测点时再预测）
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
  double fly_time = bullet_traj.fly_time;

  // 2. 获取初始瞄准角度（通过aim函数选择最佳装甲板）
  double yaw0;
  int dummy_idx;
  try {
    yaw0 = aim(target, bullet_speed, dummy_idx, false)(0);
  } catch (const std::exception & e) {
    tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
    return {false};
  }

  // 3. 计算 gimbal_delay（用于云台指令发出时刻，不影响目标状态预测）
  double gimbal_delay = std::abs(target.ekf_x()[7]) > decision_speed_ 
      ? high_speed_delay_time_ 
      : low_speed_delay_time_;

  // 4. 预计算切换点事件（仅对关键时间点附近）
  precompute_switch_events(target, bullet_speed, yaw0, fly_time);

  // 5. 计算三个关键时间点的状态
  Plan plan;
  plan.control = true;

  // t=0: 收敛检测（默认视为跟随段），预测到 0 + fly_time
  State state_origin = get_follow_state(target, bullet_speed, yaw0, 0, fly_time);
  
  // t=fire_delay: 开火判断，预测到 fire_delay + fly_time
  State state_fire = get_state_at_time(target, bullet_speed, yaw0, fire_delay_, fly_time);
  
  // t=gimbal_delay: 发送云台指令，预测到 gimbal_delay + fly_time
  State state_gimbal = get_state_at_time(target, bullet_speed, yaw0, gimbal_delay, fly_time);

  // 5. 填充plan
  plan.target_yaw = static_cast<float>(tools::limit_rad(state_gimbal.yaw + yaw0));
  plan.target_pitch = static_cast<float>(state_gimbal.pitch);
  
  plan.yaw = static_cast<float>(tools::limit_rad(state_gimbal.yaw + yaw0));
  plan.yaw_vel = static_cast<float>(state_gimbal.yaw_vel);
  plan.yaw_acc = static_cast<float>(state_gimbal.yaw_acc);
  
  plan.pitch = static_cast<float>(state_gimbal.pitch);
  plan.pitch_vel = static_cast<float>(state_gimbal.pitch_vel);
  plan.pitch_acc = static_cast<float>(state_gimbal.pitch_acc);

  // 6. 计算目标在fire_delay时刻的期望状态
  Target target_fire = target;
  target_fire.predict(fire_delay_ + fly_time);
  int fire_armor_idx;
  Eigen::Matrix<double, 2, 1> yp_fire;
  try {
    yp_fire = aim(target_fire, bullet_speed, fire_armor_idx, false);
  } catch (const std::exception & e) {
    tools::logger()->warn("Unsolvable target at fire_delay {:.2f}", bullet_speed);
    return {false};
  }
  double target_yaw_fire = yp_fire(0) - yaw0;
  double target_pitch_fire = yp_fire(1);

  // 7. 开火决策
  // 轨迹误差：规划轨迹与目标轨迹的偏差
  bool trajectory_error = std::hypot(
      state_fire.yaw - target_yaw_fire,
      state_fire.pitch - target_pitch_fire) < fire_thresh_;
  
  // 云台收敛检测：当前云台角度与期望角度的偏差
  bool gimbal_converged = skip_convergence_check_ || (
      std::hypot(
          current_yaw - tools::limit_rad(state_origin.yaw + yaw0),
          current_pitch - state_origin.pitch) < convergence_thresh_);
  
  // 有效开火区域判断
  bool fireable = true;
  if (!switch_events_.empty()) {
      int fire_idx = static_cast<int>(fire_delay_ / DT);
      bool in_transition = false;
      for (const auto& event : switch_events_) {
          if (fire_idx >= event.t_start && fire_idx <= event.t_end) {
              in_transition = true;
              break;
          }
      }
      
      if (in_transition) {
          // 过渡段：mid_ratio=1.0时考虑fire_thresh，否则不允许开火
          fireable = (mid_ratio_ >= 1.0) && trajectory_error;
      } else {
          // 跟随段
          if (mid_ratio_ >= 1.0) {
              // mid_ratio=1.0时，跟随段直接允许开火
              fireable = true;
          } else {
              // mid_ratio<1.0时，只在跟随段中间部分允许开火
              int follow_start = TIME_MIN;
              int follow_end = TIME_MAX;
              for (const auto& event : switch_events_) {
                  if (event.t_end < fire_idx) {
                      follow_start = std::max(follow_start, event.t_end);
                  }
                  if (event.t_start > fire_idx) {
                      follow_end = std::min(follow_end, event.t_start);
                  }
              }
              int follow_length = follow_end - follow_start;
              int valid_start = follow_start + static_cast<int>(follow_length * (1 - mid_ratio_) / 2);
              int valid_end = follow_end - static_cast<int>(follow_length * (1 - mid_ratio_) / 2);
              fireable = (fire_idx >= valid_start && fire_idx <= valid_end);
          }
      }
  }
  
  plan.fire = gimbal_converged && fireable;

  // 用于debug可视化：只更新一次，避免被内部扫描覆盖
  try {
    Target target_dbg = target;
    target_dbg.predict(gimbal_delay + fly_time);
    int dbg_idx;
    (void)aim(target_dbg, bullet_speed, dbg_idx, true);
  } catch (const std::exception &) {
    // 保持上一次有效debug_xyza
  }
  
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

  return plan(*target, bullet_speed, current_yaw, current_pitch);
}

Plan Planner::plan(Target target, double bullet_speed)
{
  skip_convergence_check_ = true;
  return plan(target, bullet_speed, 0.0, 0.0);
}

// ==================== 核心辅助方法 ====================
void Planner::precompute_switch_events(Target& target, double bullet_speed, double yaw0, double fly_time) {
    switch_events_.clear();
    
    // 关键时间点：fire_delay 和 gimbal_delay（high/low_speed_delay_time_）
    double key_times[] = {fire_delay_, high_speed_delay_time_, low_speed_delay_time_};
    int num_key_times = 3;
    
    // 检测4个切换点：从每个关键时间点向左和向右各搜索1个
    std::vector<std::pair<int, double>> switches;  // (切换时间idx, 角度变化量)
    int last_armor_idx = -1;
    double last_yaw = 0, last_pitch = 0;
    
    for (int i = 0; i < num_key_times; i++) {
        double key_time = key_times[i];
        int key_idx = static_cast<int>(key_time / DT);
        
        // 向左搜索最近的切换点
        for (int dt = 1; dt <= 500; dt++) {
            int t = key_idx - dt;
            if (t < TIME_MIN) break;
            
            Target target_t = target;
            target_t.predict(t * DT + fly_time);
            
            int armor_idx;
            Eigen::Matrix<double, 2, 1> yp;
            try {
                yp = aim(target_t, bullet_speed, armor_idx, false);
            } catch (...) {
                continue;
            }
            
            if (last_armor_idx != -1 && armor_idx != last_armor_idx) {
                double delta_yaw = std::abs(yp(0) - last_yaw);
                double delta_pitch = std::abs(yp(1) - last_pitch);
                double angle_change = std::sqrt(delta_yaw * delta_yaw + delta_pitch * delta_pitch);
                switches.push_back({t, angle_change});
                last_armor_idx = armor_idx;
                last_yaw = yp(0);
                last_pitch = yp(1);
                break;  // 找到后停止搜索
            }
            last_armor_idx = armor_idx;
            last_yaw = yp(0);
            last_pitch = yp(1);
        }
        
        // 重置状态，继续向右搜索
        last_armor_idx = -1;
        
        // 向右搜索最近的切换点
        for (int dt = 1; dt <= 500; dt++) {
            int t = key_idx + dt;
            if (t > TIME_MAX) break;
            
            Target target_t = target;
            target_t.predict(t * DT + fly_time);
            
            int armor_idx;
            Eigen::Matrix<double, 2, 1> yp;
            try {
                yp = aim(target_t, bullet_speed, armor_idx, false);
            } catch (...) {
                continue;
            }
            
            if (last_armor_idx != -1 && armor_idx != last_armor_idx) {
                double delta_yaw = std::abs(yp(0) - last_yaw);
                double delta_pitch = std::abs(yp(1) - last_pitch);
                double angle_change = std::sqrt(delta_yaw * delta_yaw + delta_pitch * delta_pitch);
                switches.push_back({t, angle_change});
                last_armor_idx = armor_idx;
                last_yaw = yp(0);
                last_pitch = yp(1);
                break;  // 找到后停止搜索
            }
            last_armor_idx = armor_idx;
            last_yaw = yp(0);
            last_pitch = yp(1);
        }
    }
    
    if (switches.size() < 2) return;
    
    // 按时间排序并去重
    std::sort(switches.begin(), switches.end(), 
        [](const auto& a, const auto& b) { return a.first < b.first; });
    switches.erase(std::unique(switches.begin(), switches.end(),
        [](const auto& a, const auto& b) { return a.first == b.first; }), switches.end());
    
    // max_T = 相邻切换点间距（switches已按时间排序）
    double max_T = (switches[1].first - switches[0].first) * DT;
    
    // 对每个切换点计算过渡段
    std::set<int> processed;
    for (const auto& sw : switches) {
        int chosen_switch = sw.first;
        if (processed.count(chosen_switch)) continue;
        processed.insert(chosen_switch);
        
        try {
            // 获取切换点前后的状态
            Target target_before = target;
            target_before.predict((chosen_switch - 1) * DT + fly_time);
            int dummy_idx;
            auto yp_before = aim(target_before, bullet_speed, dummy_idx, false);
            double yaw_before = yp_before(0);
            double pitch_before = yp_before(1);
            
            Target target_after = target;
            target_after.predict((chosen_switch + 1) * DT + fly_time);
            auto yp_after = aim(target_after, bullet_speed, dummy_idx, false);
            double yaw_after = yp_after(0);
            double pitch_after = yp_after(1);
            
            double delta_yaw = std::abs(yaw_after - yaw_before);
            double delta_pitch = std::abs(pitch_after - pitch_before);
            
            double min_T_yaw = std::sqrt(60 * delta_yaw / max_yaw_acc_);
            double min_T_pitch = std::sqrt(60 * delta_pitch / max_pitch_acc_);
            double min_T = std::max(min_T_yaw, min_T_pitch);
            
            double T = min_T + transition_ratio_ * (max_T - min_T);
            
            int half_steps = static_cast<int>(T / DT / 2);
            if (half_steps <= 0) continue;
            
            SwitchEvent event;
            event.t_switch = chosen_switch;
            event.t_start = std::max(TIME_MIN, chosen_switch - half_steps);
            event.t_end = std::min(TIME_MAX, chosen_switch + half_steps);
            
            Target target_start = target;
            target_start.predict(event.t_start * DT + fly_time);
            auto yp_start = aim(target_start, bullet_speed, dummy_idx, false);
            event.yaw_start = yp_start(0) - yaw0;
            event.pitch_start = yp_start(1);
            
            Target target_end = target;
            target_end.predict(event.t_end * DT + fly_time);
            auto yp_end = aim(target_end, bullet_speed, dummy_idx, false);
            event.yaw_end = yp_end(0) - yaw0;
            event.pitch_end = yp_end(1);
            
            Target target_start_prev = target;
            target_start_prev.predict((event.t_start - 1) * DT + fly_time);
            auto yp_start_prev = aim(target_start_prev, bullet_speed, dummy_idx, false);
            event.yaw_vel_start = (yp_start(0) - yp_start_prev(0)) / DT;
            event.pitch_vel_start = (yp_start(1) - yp_start_prev(1)) / DT;
            
            Target target_end_prev = target;
            target_end_prev.predict((event.t_end - 1) * DT + fly_time);
            auto yp_end_prev = aim(target_end_prev, bullet_speed, dummy_idx, false);
            event.yaw_vel_end = (yp_end(0) - yp_end_prev(0)) / DT;
            event.pitch_vel_end = (yp_end(1) - yp_end_prev(1)) / DT;
            
            double actual_T = (event.t_end - event.t_start) * DT;
            if (actual_T <= 0.001) continue;

            event.yaw_poly.solve(
                event.yaw_start, event.yaw_vel_start, 0,
                event.yaw_end, event.yaw_vel_end, 0,
                actual_T
            );
            event.pitch_poly.solve(
                event.pitch_start, event.pitch_vel_start, 0,
                event.pitch_end, event.pitch_vel_end, 0,
                actual_T
            );

            switch_events_.push_back(event);
        } catch (const std::exception& e) {
            continue;
        }
    }
}

State Planner::get_state_at_time(Target& target, double bullet_speed, double yaw0, double time, double fly_time) {
    int t = static_cast<int>(time / DT);
    
    // 检查是否在过渡段内
    for (const auto& event : switch_events_) {
        if (t >= event.t_start && t <= event.t_end) {
            double t_local = (t - event.t_start) * DT;
            
            return {
                event.yaw_poly.pos(t_local),
                event.yaw_poly.vel(t_local),
                event.yaw_poly.acc(t_local),
                event.pitch_poly.pos(t_local),
                event.pitch_poly.vel(t_local),
                event.pitch_poly.acc(t_local)
            };
        }
    }
    
    // 跟随段：直接预测
    return get_follow_state(target, bullet_speed, yaw0, t, fly_time);
}

State Planner::get_follow_state(Target& target, double bullet_speed, double yaw0, int t, double fly_time) {
    Target target_t = target;
    target_t.predict(t * DT + fly_time);
    
    // 通过aim函数选择最佳装甲板
    int armor_idx;
    Eigen::Matrix<double, 2, 1> yp;
    try {
      yp = aim(target_t, bullet_speed, armor_idx, false);
    } catch (const std::exception & e) {
        tools::logger()->warn("Unsolvable target at get_follow_state t={}", t);
        return {0, 0, 0, 0, 0, 0};
    }
    
    // 数值差分计算速度（使用相同的装甲板索引）
    Target target_t_prev = target;
    target_t_prev.predict((t - 1) * DT + fly_time);
    Eigen::Matrix<double, 2, 1> yp_prev;
    try {
      yp_prev = aim(target_t_prev, bullet_speed, armor_idx, false);
    } catch (const std::exception & e) {
        tools::logger()->warn("Unsolvable target at get_follow_state t={}", t-1);
        return {0, 0, 0, 0, 0, 0};
    }
    
    double yaw = yp(0) - yaw0;
    double yaw_vel = (yp(0) - yp_prev(0)) / DT;
    double yaw_acc = 0;  // 跟随段加速度为0
    
    double pitch = yp(1);
    double pitch_vel = (yp(1) - yp_prev(1)) / DT;
    double pitch_acc = 0;
    
    return {yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc};
}

Eigen::Matrix<double, 2, 1> Planner::aim(
  const Target & target, double bullet_speed, int & selected_armor_idx, bool update_debug_xyza)
{
  Eigen::Vector3d xyz;
  double yaw;
  auto min_dist = 1e10;
  selected_armor_idx = 0;

  const auto xyza_list = target.armor_xyza_list();

  tools::logger()->debug("[Planner::aim] Target: armor_num: {}, name: {}", target.armor_num(), static_cast<int>(target.name));
  
  // 打印 EKF 状态用于调试
  auto ekf_x = target.ekf_x();
  tools::logger()->debug("[Planner::aim] EKF state: x={:.3f}, y={:.3f}, z={:.3f}, angle={:.3f}, r={:.3f}", 
      ekf_x[0], ekf_x[2], ekf_x[4], ekf_x[6], ekf_x[8]);
  
  for (int i = 0; i < static_cast<int>(xyza_list.size()); i++) {
    const auto & xyza = xyza_list[i];
    auto dist = xyza.head<2>().norm();
    
    // 检查 bullet_traj 是否可解
    auto bullet_traj_check = tools::Trajectory(bullet_speed, dist, xyza[2]);
    tools::logger()->debug("[Planner::aim] Armor {}: dist = {:.3f}, xyz = ({:.3f}, {:.3f}, {:.3f}), angle = {:.3f}, unsolvable = {}", 
        i, dist, xyza[0], xyza[1], xyza[2], xyza[3], bullet_traj_check.unsolvable);
    
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
      yaw = xyza[3];
      selected_armor_idx = i;
    }
  }
  if (update_debug_xyza) {
    debug_xyza = Eigen::Vector4d(xyz.x(), xyz.y(), xyz.z(), yaw);
  }

  auto azim = std::atan2(xyz.y(), xyz.x());
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  if (bullet_traj.unsolvable) throw std::runtime_error("Unsolvable bullet trajectory!");

  return {tools::limit_rad(azim + yaw_offset_), -bullet_traj.pitch - pitch_offset_};
}

PlanDebug Planner::debug(Target target, double bullet_speed)
{
  PlanDebug debug_result;
  debug_result.control = true;

  // 1. 计算弹丸飞行时间
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
  double fly_time = bullet_traj.fly_time;

  // 2. 预测目标到弹丸飞行时间点
  Target target_predicted = target;
  target_predicted.predict(fly_time);

  // 3. 获取初始瞄准角度
  double yaw0;
  int dummy_idx;
  try {
    yaw0 = aim(target_predicted, bullet_speed, dummy_idx, false)(0);
  } catch (const std::exception & e) {
    tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
    debug_result.control = false;
    return debug_result;
  }

  // 4. 检测所有装甲板切换点并创建过渡事件
  std::vector<SwitchEvent> all_switch_events;
  int last_armor_idx = -1;
  
  for (int t = TIME_MIN; t <= TIME_MAX; t++) {
    Target tgt = target_predicted;
    tgt.predict(t * DT);
    
    int armor_idx;
    try {
      aim(tgt, bullet_speed, armor_idx, false);
    } catch (const std::exception & e) {
      tools::logger()->warn("Unsolvable target at debug t={}", t);
      continue;
    }
    
    if (last_armor_idx != -1 && armor_idx != last_armor_idx) {
      try {
        // 获取切换点前后的状态（用于计算最小过渡时间）
        Target target_before = target_predicted;
        target_before.predict((t - 1) * DT);
        auto yp_before = aim(target_before, bullet_speed, dummy_idx, false);
        double yaw_before = yp_before(0);
        double pitch_before = yp_before(1);
        
        Target target_after = target_predicted;
        target_after.predict((t + 1) * DT);
        auto yp_after = aim(target_after, bullet_speed, dummy_idx, false);
        double yaw_after = yp_after(0);
        double pitch_after = yp_after(1);
        
        // 计算角度变化量
        double delta_yaw = std::abs(yaw_after - yaw_before);
        double delta_pitch = std::abs(pitch_after - pitch_before);
        
        // 计算满足动力学约束的最小过渡时间
        // 五次多项式的最大加速度公式: max_acc = 60 * delta / T^2
        double min_T_yaw = std::sqrt(60 * delta_yaw / max_yaw_acc_);
        double min_T_pitch = std::sqrt(60 * delta_pitch / max_pitch_acc_);
        double min_T = std::max(min_T_yaw, min_T_pitch);
        
        // 根据transition_ratio放大过渡时间（0.0=最小过渡, 1.0=最大过渡）
        double max_T = 0.15;  // 150ms最大过渡时间
        double T = min_T + transition_ratio_ * (max_T - min_T);
        
        // 以切换点为中心对称分布过渡段
        SwitchEvent event;
        int half_steps = static_cast<int>(T / DT / 2);
        if (half_steps <= 0) {
          continue;
        }
        event.t_switch = t;
        event.t_start = std::max(TIME_MIN, t - half_steps);
        event.t_end = std::min(TIME_MAX, t + half_steps);
        
        // 获取过渡段起点状态（切换点前的装甲板）
        Target target_start = target_predicted;
        target_start.predict(event.t_start * DT);
        auto yp_start = aim(target_start, bullet_speed, dummy_idx, false);
        event.yaw_start = yp_start(0) - yaw0;
        event.pitch_start = yp_start(1);
        
        // 获取过渡段终点状态（切换点后的装甲板）
        Target target_end = target_predicted;
        target_end.predict(event.t_end * DT);
        auto yp_end = aim(target_end, bullet_speed, dummy_idx, false);
        event.yaw_end = yp_end(0) - yaw0;
        event.pitch_end = yp_end(1);
        
        // 数值差分计算速度
        Target target_start_prev = target_predicted;
        target_start_prev.predict((event.t_start - 1) * DT);
        auto yp_start_prev = aim(target_start_prev, bullet_speed, dummy_idx, false);
        event.yaw_vel_start = (yp_start(0) - yp_start_prev(0)) / DT;
        event.pitch_vel_start = (yp_start(1) - yp_start_prev(1)) / DT;
        
        Target target_end_prev = target_predicted;
        target_end_prev.predict((event.t_end - 1) * DT);
        auto yp_end_prev = aim(target_end_prev, bullet_speed, dummy_idx, false);
        event.yaw_vel_end = (yp_end(0) - yp_end_prev(0)) / DT;
        event.pitch_vel_end = (yp_end(1) - yp_end_prev(1)) / DT;
        
        // 生成五次多项式
        double actual_T = (event.t_end - event.t_start) * DT;
        if (actual_T <= 0.001) {
          continue;
        }

        event.yaw_poly.solve(
          event.yaw_start, event.yaw_vel_start, 0,
          event.yaw_end, event.yaw_vel_end, 0,
          actual_T
        );
        event.pitch_poly.solve(
          event.pitch_start, event.pitch_vel_start, 0,
          event.pitch_end, event.pitch_vel_end, 0,
          actual_T
        );

        all_switch_events.push_back(event);
      } catch (const std::exception & e) {
        tools::logger()->warn("Unsolvable target at debug switch t={}", t);
      }
    }
    last_armor_idx = armor_idx;
  }

  // 5. 遍历所有时间点，计算规划轨迹和目标轨迹
  for (int t_ms = -10; t_ms <= 500; t_ms += 10) {
    double time = t_ms * 0.001;
    int t_idx = static_cast<int>(time / DT);
    
    double plan_yaw, plan_pitch;
    bool in_transition = false;
    
    // 检查是否在过渡段内
    for (const auto& event : all_switch_events) {
      if (t_idx >= event.t_start && t_idx <= event.t_end) {
        double t_local = (t_idx - event.t_start) * DT;
        plan_yaw = tools::limit_rad(event.yaw_poly.pos(t_local) + yaw0);
        plan_pitch = event.pitch_poly.pos(t_local);
        in_transition = true;
        break;
      }
    }
    
    // 跟随段：直接预测
    if (!in_transition) {
      Target tgt = target_predicted;
      tgt.predict(time);
      int armor_idx;
      auto yp = aim(tgt, bullet_speed, armor_idx, false);
      plan_yaw = tools::limit_rad(yp(0));
      plan_pitch = yp(1);
    }
    
    // 获取目标状态
    Target target_t = target;
    target_t.predict(fly_time + time);
    int armor_idx;
    auto yp = aim(target_t, bullet_speed, armor_idx, false);
    double target_yaw = tools::limit_rad(yp(0));
    double target_pitch = yp(1);

    debug_result.target_yaw_list.push_back(static_cast<float>(target_yaw));
    debug_result.target_pitch_list.push_back(static_cast<float>(target_pitch));
    debug_result.plan_yaw_list.push_back(static_cast<float>(plan_yaw));
    debug_result.plan_pitch_list.push_back(static_cast<float>(plan_pitch));

    // 计算是否允许发射
    bool trajectory_error = std::hypot(target_yaw - plan_yaw, target_pitch - plan_pitch) < fire_thresh_;
    
    bool fireable = false;
    in_transition = false;
    
    // 检查是否在过渡段内
    for (const auto& event : all_switch_events) {
      if (t_idx >= event.t_start && t_idx <= event.t_end) {
        in_transition = true;
        break;
      }
    }
    
    if (in_transition) {
      // 过渡段：mid_ratio=1.0时考虑fire_thresh，否则不允许开火
      fireable = (mid_ratio_ >= 1.0) && trajectory_error;
    } else {
      // 跟随段
      if (mid_ratio_ >= 1.0) {
        // mid_ratio=1.0时，跟随段直接允许开火
        fireable = true;
      } else {
        // mid_ratio<1.0时，只在跟随段中间部分允许开火
        int follow_start = TIME_MIN;
        int follow_end = TIME_MAX;
        for (const auto& event : all_switch_events) {
          if (event.t_end < t_idx) {
            follow_start = std::max(follow_start, event.t_end);
          }
          if (event.t_start > t_idx) {
            follow_end = std::min(follow_end, event.t_start);
          }
        }
        int follow_length = follow_end - follow_start;
        int valid_start = follow_start + static_cast<int>(follow_length * (1 - mid_ratio_) / 2);
        int valid_end = follow_end - static_cast<int>(follow_length * (1 - mid_ratio_) / 2);
        fireable = (t_idx >= valid_start && t_idx <= valid_end);
      }
    }
    
    debug_result.fireable_list.push_back(fireable);
  }

  return debug_result;
}

}  // namespace auto_aim