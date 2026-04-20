#include "planner.hpp"

#include <vector>

#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace auto_aim
{
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
  skip_convergence_check_ = false;

  setup_yaw_solver(config_path);
  setup_pitch_solver(config_path);
}

Plan Planner::plan(Target target, double bullet_speed, double current_yaw, double current_pitch)
{
  // 0. Check bullet speed
  // if (bullet_speed < 10 || bullet_speed > 25) {
  //   bullet_speed = 22;
  // }

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
  double yaw0;
  Trajectory traj;
  try {
    int dummy_idx;
    yaw0 = aim(target, bullet_speed, dummy_idx)(0);
    traj = get_trajectory(target, yaw0, bullet_speed);
  } catch (const std::exception & e) {
    tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
    return {false};
  }

  // 3. Solve yaw
  Eigen::VectorXd x0(2);
  x0 << traj(0, 0), traj(1, 0);
  tiny_set_x0(yaw_solver_, x0);

  yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
  tiny_solve(yaw_solver_);

  // 4. Solve pitch
  x0 << traj(2, 0), traj(3, 0);
  tiny_set_x0(pitch_solver_, x0);

  pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  tiny_solve(pitch_solver_);

  Plan plan;
  plan.control = true;

  plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
  plan.target_pitch = traj(2, HALF_HORIZON);

  plan.yaw = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
  plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
  plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);

  plan.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
  plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
  plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

  int shoot_offset = static_cast<int>(fire_delay_ / DT);
  bool trajectory_error = std::hypot(
      traj(0, shoot_offset) - yaw_solver_->work->x(0, shoot_offset),
      traj(2, shoot_offset) -
        pitch_solver_->work->x(0, shoot_offset)) < fire_thresh_;
  
  bool gimbal_converged = skip_convergence_check_ || (
      std::hypot(
          current_yaw - traj(0, 0),
          current_pitch - traj(2, 0)) < convergence_thresh_);
  
  // 确定shoot_offset所在的跟随段
  int current_follow_start = 0;
  int current_follow_end = HORIZON - 1;
  bool found_switch_points = false;
  
  // 找到左边最近的切换点
  for (int i = shoot_offset - 1; i >= 0; i--) {
      if (switch_points_[i]) {
          current_follow_start = i + 1;
          found_switch_points = true;
          break;
      }
  }
  
  // 找到右边最近的切换点
  for (int i = shoot_offset + 1; i < HORIZON; i++) {
      if (switch_points_[i]) {
          current_follow_end = i - 1;
          found_switch_points = true;
          break;
      }
  }
  
  // 计算有效发射区域
  bool in_valid_fire_region = true; // 默认允许发射
  if (found_switch_points) {
      int follow_length = current_follow_end - current_follow_start;
      int mid_start = current_follow_start + follow_length * (1 - mid_ratio_) / 2;
      int mid_end = current_follow_end - follow_length * (1 - mid_ratio_) / 2;
      in_valid_fire_region = (shoot_offset >= mid_start && shoot_offset <= mid_end);
  }
  
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

//  skip_convergence_check_ = false;
  return plan(*target, bullet_speed, current_yaw, current_pitch);
}

Plan Planner::plan(Target target, double bullet_speed)
{
  skip_convergence_check_ = true;
  return plan(target, bullet_speed, 0.0, 0.0);
}

void Planner::setup_yaw_solver(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto max_yaw_acc = tools::read<double>(yaml, "max_yaw_acc");
  auto Q_yaw = tools::read<std::vector<double>>(yaml, "Q_yaw");
  auto R_yaw = tools::read<std::vector<double>>(yaml, "R_yaw");

  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q(Q_yaw.data());
  Eigen::Matrix<double, 1, 1> R(R_yaw.data());
  tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_yaw_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_yaw_acc);
  tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);

  yaw_solver_->settings->max_iter = 10;
}

void Planner::setup_pitch_solver(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto max_pitch_acc = tools::read<double>(yaml, "max_pitch_acc");
  auto Q_pitch = tools::read<std::vector<double>>(yaml, "Q_pitch");
  auto R_pitch = tools::read<std::vector<double>>(yaml, "R_pitch");

  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q(Q_pitch.data());
  Eigen::Matrix<double, 1, 1> R(R_pitch.data());
  tiny_setup(&pitch_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_pitch_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_pitch_acc);
  tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);

  pitch_solver_->settings->max_iter = 10;
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

  target.predict(DT);  // [0] = -HALF_HORIZON * DT -> [HHALF_HORIZON] = 0
  int current_selected_armor = 0;
  auto yaw_pitch = aim(target, bullet_speed, current_selected_armor);
  
  switch_points_.resize(HORIZON, 0);

  for (int i = 0; i < HORIZON; i++) {
    target.predict(DT);
    int next_selected_armor = 0;
    auto yaw_pitch_next = aim(target, bullet_speed, next_selected_armor);

    // 计算角度差时考虑连续性，避免角度跳变
    double yaw_diff = yaw_pitch_next(0) - yaw_pitch_last(0);
    if (yaw_diff > CV_PI) yaw_diff -= 2 * CV_PI;
    if (yaw_diff < -CV_PI) yaw_diff += 2 * CV_PI;
    auto yaw_vel = yaw_diff / (2 * DT);
    
    auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

    // 计算相对yaw时也考虑连续性
    double relative_yaw = yaw_pitch(0) - yaw0;
    if (relative_yaw > CV_PI) relative_yaw -= 2 * CV_PI;
    if (relative_yaw < -CV_PI) relative_yaw += 2 * CV_PI;
    
    traj.col(i) << relative_yaw, yaw_vel, yaw_pitch(1), pitch_vel;

    // 检测装甲板切换点
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

  double yaw0;
  Trajectory traj;
  try {
    int dummy_idx;
    yaw0 = aim(target, bullet_speed, dummy_idx)(0);
    traj = get_trajectory(target, yaw0, bullet_speed);
  } catch (const std::exception & e) {
    tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
    debug_result.control = false;
    return debug_result;
  }

  Eigen::VectorXd x0(2);
  x0 << traj(0, 0), traj(1, 0);
  tiny_set_x0(yaw_solver_, x0);

  yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
  tiny_solve(yaw_solver_);

  x0 << traj(2, 0), traj(3, 0);
  tiny_set_x0(pitch_solver_, x0);

  pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  tiny_solve(pitch_solver_);

  int shoot_offset = static_cast<int>(fire_delay_ / DT);

  for (int i = 0; i < HORIZON; i++) {
    double plan_yaw = tools::limit_rad(yaw_solver_->work->x(0, i) + yaw0);
    double plan_pitch = pitch_solver_->work->x(0, i);
    double target_yaw = tools::limit_rad(traj(0, i) + yaw0);
    double target_pitch = traj(2, i);

    debug_result.target_yaw_list.push_back(static_cast<float>(target_yaw));
    debug_result.target_pitch_list.push_back(static_cast<float>(target_pitch));
    debug_result.plan_yaw_list.push_back(static_cast<float>(plan_yaw));
    debug_result.plan_pitch_list.push_back(static_cast<float>(plan_pitch));

    bool trajectory_error = std::hypot(
        traj(0, i) - yaw_solver_->work->x(0, i),
        traj(2, i) - pitch_solver_->work->x(0, i)) < fire_thresh_;

    int current_follow_start = 0;
    int current_follow_end = HORIZON - 1;
    bool found_switch_points = false;

    for (int j = shoot_offset - 1; j >= 0; j--) {
      if (switch_points_[j]) {
        current_follow_start = j + 1;
        found_switch_points = true;
        break;
      }
    }

    for (int j = shoot_offset + 1; j < HORIZON; j++) {
      if (switch_points_[j]) {
        current_follow_end = j - 1;
        found_switch_points = true;
        break;
      }
    }

    bool in_valid_fire_region = true;
    if (found_switch_points) {
      int follow_length = current_follow_end - current_follow_start;
      int mid_start = current_follow_start + follow_length * (1 - mid_ratio_) / 2;
      int mid_end = current_follow_end - follow_length * (1 - mid_ratio_) / 2;
      in_valid_fire_region = (i >= mid_start && i <= mid_end);
    }

    debug_result.fireable_list.push_back(trajectory_error && in_valid_fire_region);
  }

  return debug_result;
}

}  // namespace auto_aim