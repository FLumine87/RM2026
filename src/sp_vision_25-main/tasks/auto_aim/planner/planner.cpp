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

  // 2. 获取初始瞄准角度（通过aim函数选择最佳装甲板）
  double yaw0;
  int dummy_idx;
  try {
    yaw0 = aim(target, bullet_speed, dummy_idx)(0);
  } catch (const std::exception & e) {
    tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
    return {false};
  }

  // 3. 预计算切换点事件（仅对关键时间点附近，最多4个）
  precompute_switch_events(target, bullet_speed, yaw0);

  // 4. 计算三个关键时间点的状态
  Plan plan;
  plan.control = true;

  // t=0: 收敛检测（默认视为跟随段）
  State state_origin = get_follow_state(target, bullet_speed, yaw0, 0);
  
  // t=fire_delay: 开火判断
  State state_fire = get_state_at_time(target, bullet_speed, yaw0, fire_delay_);
  
  // t=gimbal_delay: 发送云台指令
  double gimbal_delay = std::abs(target.ekf_x()[7]) > decision_speed_ 
      ? high_speed_delay_time_ 
      : low_speed_delay_time_;
  State state_gimbal = get_state_at_time(target, bullet_speed, yaw0, gimbal_delay);

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
  target_fire.predict(fire_delay_);
  int fire_armor_idx;
  auto yp_fire = aim(target_fire, bullet_speed, fire_armor_idx);
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

// ==================== 核心辅助方法 ====================
void Planner::precompute_switch_events(Target& target, double bullet_speed, double yaw0) {
    switch_events_.clear();
    
    // 获取目标当前角速度
    double current_yaw_vel = std::abs(target.ekf_x()[7]);
    double current_pitch_vel = std::abs(target.ekf_x()[9]);
    
    // 关键时间点：原点(收敛检测)、fire_delay(开火判断)、两个云台延迟(发送指令)
    double key_times[] = {0.0, fire_delay_, high_speed_delay_time_, low_speed_delay_time_};
    int num_key_times = sizeof(key_times) / sizeof(key_times[0]);
    
    // 记录所有装甲板切换点及其角度变化
    std::vector<std::pair<int, double>> all_switch_points;  // (切换时间, 角度变化量)
    int last_armor_idx = -1;
    double last_yaw = 0, last_pitch = 0;
    
    tools::logger()->debug("[Planner::precompute_switch_events] Starting to detect switch points...");
    
    for (int t = TIME_MIN; t <= TIME_MAX; t++) {
        Target target_t = target;
        target_t.predict(t * DT);
        
        int armor_idx;
        auto yp = aim(target_t, bullet_speed, armor_idx);
        double yaw = yp(0);
        double pitch = yp(1);
        
        if (last_armor_idx != -1 && armor_idx != last_armor_idx) {
            double delta_yaw = std::abs(yaw - last_yaw);
            double delta_pitch = std::abs(pitch - last_pitch);
            double angle_change = std::sqrt(delta_yaw * delta_yaw + delta_pitch * delta_pitch);
            all_switch_points.push_back({t, angle_change});
            tools::logger()->debug("[Planner::precompute_switch_events] Detected switch at t={}: {} -> {}, angle_change={:.3f}", t, last_armor_idx, armor_idx, angle_change);
        }
        last_armor_idx = armor_idx;
        last_yaw = yaw;
        last_pitch = pitch;
    }
    
    tools::logger()->debug("[Planner::precompute_switch_events] Total switch points detected: {}", all_switch_points.size());
    
    // 从每个关键时间点往左右找最近的切换点，共处理4个
    std::set<int> selected_switches;
    
    for (int i = 0; i < num_key_times && static_cast<int>(selected_switches.size()) < 4; i++) {
        double key_time = key_times[i];
        int key_idx = static_cast<int>(key_time / DT);
        
        // 往左右搜索最近的切换点
        int left_switch = -1, right_switch = -1;
        for (int dt = 0; dt <= 500 && (left_switch == -1 || right_switch == -1); dt++) {
            if (left_switch == -1) {
                for (const auto& switch_pair : all_switch_points) {
                    if (switch_pair.first == key_idx - dt) {
                        left_switch = switch_pair.first;
                        break;
                    }
                }
            }
            if (right_switch == -1) {
                for (const auto& switch_pair : all_switch_points) {
                    if (switch_pair.first == key_idx + dt) {
                        right_switch = switch_pair.first;
                        break;
                    }
                }
            }
        }
        
        // 选择最近的切换点
        int chosen_switch = -1;
        if (left_switch != -1 && right_switch != -1) {
            chosen_switch = (key_idx - left_switch <= right_switch - key_idx) ? left_switch : right_switch;
        } else if (left_switch != -1) {
            chosen_switch = left_switch;
        } else if (right_switch != -1) {
            chosen_switch = right_switch;
        }
        
        if (chosen_switch != -1 && selected_switches.count(chosen_switch) == 0) {
            // 获取切换点前后的状态（用于计算最小过渡时间）
            Target target_before = target;
            target_before.predict((chosen_switch - 1) * DT);
            int dummy_idx;
            auto yp_before = aim(target_before, bullet_speed, dummy_idx);
            double yaw_before = yp_before(0);
            double pitch_before = yp_before(1);
            
            Target target_after = target;
            target_after.predict((chosen_switch + 1) * DT);
            auto yp_after = aim(target_after, bullet_speed, dummy_idx);
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
            event.t_start = std::max(TIME_MIN, chosen_switch - half_steps);
            event.t_end = std::min(TIME_MAX, chosen_switch + half_steps);
            
            // 获取过渡段起点状态
            Target target_start = target;
            target_start.predict(event.t_start * DT);
            auto yp_start = aim(target_start, bullet_speed, dummy_idx);
            event.yaw_start = yp_start(0) - yaw0;
            event.pitch_start = yp_start(1);
            
            // 获取过渡段终点状态
            Target target_end = target;
            target_end.predict(event.t_end * DT);
            auto yp_end = aim(target_end, bullet_speed, dummy_idx);
            event.yaw_end = yp_end(0) - yaw0;
            event.pitch_end = yp_end(1);
            
            // 数值差分计算速度
            Target target_start_prev = target;
            target_start_prev.predict((event.t_start - 1) * DT);
            auto yp_start_prev = aim(target_start_prev, bullet_speed, dummy_idx);
            event.yaw_vel_start = (yp_start(0) - yp_start_prev(0)) / DT;
            event.pitch_vel_start = (yp_start(1) - yp_start_prev(1)) / DT;
            
            Target target_end_prev = target;
            target_end_prev.predict((event.t_end - 1) * DT);
            auto yp_end_prev = aim(target_end_prev, bullet_speed, dummy_idx);
            event.yaw_vel_end = (yp_end(0) - yp_end_prev(0)) / DT;
            event.pitch_vel_end = (yp_end(1) - yp_end_prev(1)) / DT;
            
            // 生成五次多项式
            double actual_T = (event.t_end - event.t_start) * DT;
            if (actual_T > 0.001) {
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
            }
            
            switch_events_.push_back(event);
        }
    }
}

State Planner::get_state_at_time(Target& target, double bullet_speed, double yaw0, double time) {
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
    return get_follow_state(target, bullet_speed, yaw0, t);
}

State Planner::get_follow_state(Target& target, double bullet_speed, double yaw0, int t) {
    Target target_t = target;
    target_t.predict(t * DT);
    
    // 通过aim函数选择最佳装甲板
    int armor_idx;
    auto yp = aim(target_t, bullet_speed, armor_idx);
    
    // 数值差分计算速度（使用相同的装甲板索引）
    Target target_t_prev = target;
    target_t_prev.predict((t - 1) * DT);
    auto yp_prev = aim(target_t_prev, bullet_speed, armor_idx);
    
    double yaw = yp(0) - yaw0;
    double yaw_vel = (yp(0) - yp_prev(0)) / DT;
    double yaw_acc = 0;  // 跟随段加速度为0
    
    double pitch = yp(1);
    double pitch_vel = (yp(1) - yp_prev(1)) / DT;
    double pitch_acc = 0;
    
    return {yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc};
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
    yaw0 = aim(target_predicted, bullet_speed, dummy_idx)(0);
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
    aim(tgt, bullet_speed, armor_idx);
    
    if (last_armor_idx != -1 && armor_idx != last_armor_idx) {
      // 获取切换点前后的状态（用于计算最小过渡时间）
      Target target_before = target_predicted;
      target_before.predict((t - 1) * DT);
      auto yp_before = aim(target_before, bullet_speed, dummy_idx);
      double yaw_before = yp_before(0);
      double pitch_before = yp_before(1);
      
      Target target_after = target_predicted;
      target_after.predict((t + 1) * DT);
      auto yp_after = aim(target_after, bullet_speed, dummy_idx);
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
      event.t_start = std::max(TIME_MIN, t - half_steps);
      event.t_end = std::min(TIME_MAX, t + half_steps);
      
      // 获取过渡段起点状态（切换点前的装甲板）
      Target target_start = target_predicted;
      target_start.predict(event.t_start * DT);
      auto yp_start = aim(target_start, bullet_speed, dummy_idx);
      event.yaw_start = yp_start(0) - yaw0;
      event.pitch_start = yp_start(1);
      
      // 获取过渡段终点状态（切换点后的装甲板）
      Target target_end = target_predicted;
      target_end.predict(event.t_end * DT);
      auto yp_end = aim(target_end, bullet_speed, dummy_idx);
      event.yaw_end = yp_end(0) - yaw0;
      event.pitch_end = yp_end(1);
      
      // 数值差分计算速度
      Target target_start_prev = target_predicted;
      target_start_prev.predict((event.t_start - 1) * DT);
      auto yp_start_prev = aim(target_start_prev, bullet_speed, dummy_idx);
      event.yaw_vel_start = (yp_start(0) - yp_start_prev(0)) / DT;
      event.pitch_vel_start = (yp_start(1) - yp_start_prev(1)) / DT;
      
      Target target_end_prev = target_predicted;
      target_end_prev.predict((event.t_end - 1) * DT);
      auto yp_end_prev = aim(target_end_prev, bullet_speed, dummy_idx);
      event.yaw_vel_end = (yp_end(0) - yp_end_prev(0)) / DT;
      event.pitch_vel_end = (yp_end(1) - yp_end_prev(1)) / DT;
      
      // 生成五次多项式
      double actual_T = (event.t_end - event.t_start) * DT;
      if (actual_T > 0.001) {
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
      }
      
      all_switch_events.push_back(event);
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
      auto yp = aim(tgt, bullet_speed, armor_idx);
      plan_yaw = tools::limit_rad(yp(0));
      plan_pitch = yp(1);
    }
    
    // 获取目标状态
    Target target_t = target;
    target_t.predict(fly_time + time);
    int armor_idx;
    auto yp = aim(target_t, bullet_speed, armor_idx);
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