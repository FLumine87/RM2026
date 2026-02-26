#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/bullet_detector/aim_corrector.hpp"
#include "tasks/bullet_detector/detect_bullet.hpp"
#include "tools/coord_converter.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明 }"
  "{config-path c  | configs/demo.yaml | yaml配置文件的路径}"
  "{start-index s  | 0                 | 视频起始帧下标    }"
  "{end-index e    | 0                 | 视频结束帧下标    }"
  "{@input-path    | assets/demo/demo  | avi和txt文件的路径}";

// 全局弹丸参数
const double BULLET_V0 = 20.0;         // 弹丸初速度 (m/s)
const double BULLET_RESISTANCE_K = 0.022;  // 空气阻力系数
const double GRAVITY_ACCELERATION = 9.79;  // 重力加速度 (m/s²)
const double BULLET_RADIUS = 0.0085;     // 弹丸半径 (m)

// 弹道轨迹结构体
struct BulletTrajectory {
  int id;                               // 轨迹ID
  double fire_time;                     // 发射时间
  Eigen::Vector3d fire_direction;       // 发射方向
  double bullet_v0;                     // 弹丸初速度
  double last_detected_time;            // 最后检测到的时间
  std::vector<cv::Point2f> trajectory_points;  // 轨迹点（图像坐标）
  std::vector<cv::Point2f> bullet_history;     // 弹丸历史位置（图像坐标）
  double alpha;                         // 轨迹透明度
  
  // 构造函数
  BulletTrajectory(int id, double fire_time, const Eigen::Vector3d& fire_direction, double bullet_v0, double current_time) 
    : id(id), fire_time(fire_time), fire_direction(fire_direction), bullet_v0(bullet_v0), 
      last_detected_time(current_time), alpha(1.0) {}
};

// 轨迹存储容器
std::vector<BulletTrajectory> bullet_trajectories;

// 计算最佳发射时间
double find_best_fire_time(
  aimer::CoordConverter* converter,
  const aimer::aim::ImageBullet& detected_bullet,
  const Eigen::Vector3d& fire_direction,
  double bullet_speed) {
  
  // 将检测到的弹丸转换为现实坐标系
  cv::Point2f undistorted_center = converter->pd_to_pu(detected_bullet.center);
  // 使用默认距离 10.0 米（假设弹丸在这个距离附近）
  double assumed_distance = 10.0;
  Eigen::Vector3d detected_xyz_c = converter->pu_to_pc(undistorted_center, assumed_distance);
  Eigen::Vector3d detected_xyz_i = converter->pc_to_pi(detected_xyz_c);
  
  // 搜索最佳发射时间（过去0.5秒内）
  double best_fire_time = converter->get_img_t() - 0.25;
  double min_error = std::numeric_limits<double>::max();
  
  for (double t = converter->get_img_t() - 0.5; t < converter->get_img_t(); t += 0.01) {
    // 假设此时发射，计算弹丸当前位置（简化模型：直线飞行）
    double flight_time = converter->get_img_t() - t;
    Eigen::Vector3d predicted_pos = fire_direction * bullet_speed * flight_time;
    
    // 计算误差
    double error = (predicted_pos - detected_xyz_i).norm();
    if (error < min_error) {
      min_error = error;
      best_fire_time = t;
    }
  }
  
  return best_fire_time;
}

// 创建模拟弹丸
void create_simulated_bullet(
  aimer::aim::AimCorrector& aim_corrector,
  const double& fire_time,
  const Eigen::Vector3d& fire_direction,
  const double& target_distance,
  int bullet_id,
  double bullet_v0) {
  
  Eigen::Vector3d target_pos = fire_direction * target_distance;
  
  aimer::AimInfo aim_info;
  aim_info.shoot_param.v0 = bullet_v0;  // 弹丸初速度
  aim_info.shoot_param.aim_angle = 0.0;  // 水平发射
  aim_info.shoot_param.target_xyz_i_camera = target_pos;
  
  aimer::aim::IdTLatencyAimCorrection aim_correction;
  aim_correction.id = bullet_id;  // 使用唯一ID
  aim_correction.img_t = fire_time;
  aim_correction.img_to_predict_latency = 0.0;
  aim_correction.aim = aim_info;
  aim_correction.correction = Eigen::Vector2d(0.0, 0.0);
  
  aim_corrector.add_aim(aim_correction);
  aim_corrector.update_bullet_id(bullet_id);
}

// 计算弹丸与预测轨迹的偏差
Eigen::Vector3d calculate_bullet_error(
  aimer::CoordConverter& converter,
  const aimer::aim::ImageBullet& detected_bullet,
  const aimer::aim::IdPos& predicted_bullet) {
  
  // 将检测到的弹丸转换为现实坐标系
  cv::Point2f undistorted_center = converter.pd_to_pu(detected_bullet.center);
  // 使用默认距离 10.0 米（假设弹丸在这个距离附近）
  double assumed_distance = 10.0;
  Eigen::Vector3d detected_xyz_c = converter.pu_to_pc(undistorted_center, assumed_distance);
  Eigen::Vector3d detected_xyz_i = converter.pc_to_pi(detected_xyz_c);
  
  // 计算偏差
  return detected_xyz_i - predicted_bullet.pos;
}

// 抛物线拟合结构体
struct ParabolaFit {
  double a, b, c;  // y = a*x² + b*x + c
  
  ParabolaFit() : a(0), b(0), c(0) {}
  
  // 使用最小二乘法拟合抛物线
  void fit(const std::vector<cv::Point2f>& points) {
    if (points.size() < 3) return;
    
    int n = points.size();
    double sum_x = 0, sum_y = 0, sum_x2 = 0, sum_x3 = 0, sum_x4 = 0, sum_xy = 0;
    
    for (const auto& p : points) {
      double x = p.x;
      double y = p.y;
      sum_x += x;
      sum_y += y;
      sum_x2 += x * x;
      sum_x3 += x * x * x;
      sum_x4 += x * x * x * x;
      sum_xy += x * y;
    }
    
    // 构建矩阵
    double A = n * sum_x2 - sum_x * sum_x;
    double B = n * sum_x3 - sum_x * sum_x2;
    double C = sum_x2 * sum_x2 - sum_x * sum_x3;
    double D = n * sum_xy - sum_x * sum_y;
    double E = sum_x2 * sum_xy - sum_x * sum_y * sum_x;
    
    // 计算系数
    if (A * C - B * B != 0) {
      a = (D * B - E * A) / (A * C - B * B);
      b = (E * C - D * B) / (A * C - B * B);
      c = (sum_y - a * sum_x2 - b * sum_x) / n;
    }
  }
  
  // 根据x计算y
  double get_y(double x) const {
    return a * x * x + b * x + c;
  }
};

// 计算弹道轨迹点（基于弹丸历史位置和更平滑的曲线拟合）
std::vector<cv::Point2f> calculate_trajectory_points(
  aimer::CoordConverter& converter,
  double fire_time,
  double current_time,
  const Eigen::Vector3d& fire_direction,
  double bullet_v0,
  std::vector<cv::Point2f>& bullet_history) {
  std::vector<cv::Point2f> trajectory_points;
  
  // 输出图像尺寸
  int img_width = converter.get_img_ref().cols;
  int img_height = converter.get_img_ref().rows;
  tools::logger()->info("Image size: {}x{}", img_width, img_height);
  
  // 异常值检测：排除与默认弹道相差较大的检测结果
  if (bullet_history.size() >= 3) {
    // 计算历史位置的统计信息
    double mean_x = 0, mean_y = 0;
    for (const auto& p : bullet_history) {
      mean_x += p.x;
      mean_y += p.y;
    }
    mean_x /= bullet_history.size();
    mean_y /= bullet_history.size();
    
    // 计算标准差
    double std_x = 0, std_y = 0;
    for (const auto& p : bullet_history) {
      std_x += std::pow(p.x - mean_x, 2);
      std_y += std::pow(p.y - mean_y, 2);
    }
    std_x = std::sqrt(std_x / bullet_history.size());
    std_y = std::sqrt(std_y / bullet_history.size());
    
    // 定义异常值阈值（例如，2.5倍标准差，更严格的过滤）
    const double threshold = 2.5;
    
    // 过滤异常值
    std::vector<cv::Point2f> filtered_history;
    for (const auto& p : bullet_history) {
      double z_x = std::abs(p.x - mean_x) / std_x;
      double z_y = std::abs(p.y - mean_y) / std_y;
      if (z_x < threshold && z_y < threshold) {
        filtered_history.push_back(p);
      } else {
        tools::logger()->info("Filtered outlier: ({}, {}) with z-score ({:.2f}, {:.2f})\n", p.x, p.y, z_x, z_y);
      }
    }
    
    // 使用过滤后的历史位置
    if (!filtered_history.empty()) {
      bullet_history = filtered_history;
      tools::logger()->info("Filtered bullet history size: {}", bullet_history.size());
    }
  }
  
  // 如果有足够的历史位置，基于这些位置计算轨迹
  if (bullet_history.size() >= 2) {
    // 输出历史位置
    tools::logger()->info("Bullet history size: {}", bullet_history.size());
    for (size_t i = 0; i < bullet_history.size(); i++) {
      tools::logger()->info("Bullet history [{}]: ({}, {})", i, bullet_history[i].x, bullet_history[i].y);
    }
    
    // 按x坐标排序历史位置
    std::sort(bullet_history.begin(), bullet_history.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
      return a.x < b.x;
    });
    
    // 进一步过滤：移除与前一个点距离过大的点
    std::vector<cv::Point2f> final_history;
    if (!bullet_history.empty()) {
      final_history.push_back(bullet_history[0]);
      for (size_t i = 1; i < bullet_history.size(); i++) {
        double distance = std::sqrt(std::pow(bullet_history[i].x - bullet_history[i-1].x, 2) + 
                                    std::pow(bullet_history[i].y - bullet_history[i-1].y, 2));
        // 定义距离阈值，例如，图像宽度的10%
        double distance_threshold = img_width * 0.1;
        if (distance < distance_threshold) {
          final_history.push_back(bullet_history[i]);
        } else {
          tools::logger()->info("Filtered distant point: ({}, {}) with distance {:.2f} from ({}, {})\n", 
                               bullet_history[i].x, bullet_history[i].y, distance, 
                               bullet_history[i-1].x, bullet_history[i-1].y);
        }
      }
    }
    
    // 使用过滤后的最终历史位置
    if (!final_history.empty()) {
      bullet_history = final_history;
      tools::logger()->info("Final bullet history size: {}", bullet_history.size());
      for (size_t i = 0; i < bullet_history.size(); i++) {
        tools::logger()->info("Final bullet history [{}]: ({}, {})", i, bullet_history[i].x, bullet_history[i].y);
      }
    }
    
    // 使用更简单的方法生成平滑的轨迹：线性插值
    int num_points = 100; // 生成更多的轨迹点，使轨迹更平滑
    
    if (bullet_history.size() == 2) {
      // 两点之间使用直线
      for (int i = 0; i <= num_points; i++) {
        float t = static_cast<float>(i) / num_points;
        float x = bullet_history[0].x * (1 - t) + bullet_history[1].x * t;
        float y = bullet_history[0].y * (1 - t) + bullet_history[1].y * t;
        
        cv::Point2f pixel_point(x, y);
        if (pixel_point.x >= 0 && pixel_point.y >= 0 && pixel_point.x < img_width && pixel_point.y < img_height) {
          trajectory_points.push_back(pixel_point);
        }
      }
    } else if (bullet_history.size() > 2) {
      // 多点之间使用线性插值
      for (size_t i = 0; i < bullet_history.size() - 1; i++) {
        int segment_points = num_points / (bullet_history.size() - 1);
        for (int j = 0; j <= segment_points; j++) {
          float t = static_cast<float>(j) / segment_points;
          float x = bullet_history[i].x * (1 - t) + bullet_history[i+1].x * t;
          float y = bullet_history[i].y * (1 - t) + bullet_history[i+1].y * t;
          
          cv::Point2f pixel_point(x, y);
          if (pixel_point.x >= 0 && pixel_point.y >= 0 && pixel_point.x < img_width && pixel_point.y < img_height) {
            trajectory_points.push_back(pixel_point);
          }
        }
      }
    }
    
    // 输出轨迹点数量
    tools::logger()->info("Calculated {} trajectory points from linear interpolation", trajectory_points.size());
  } else if (!bullet_history.empty()) {
    // 如果只有一个历史位置，生成从图像底部到该位置的轨迹
    cv::Point2f bullet_center = bullet_history.back();
    
    // 生成从图像底部到弹丸位置的轨迹
    int num_points = 50; // 生成更多的轨迹点，使轨迹更平滑
    for (int i = 0; i <= num_points; i++) {
      float t = static_cast<float>(i) / num_points;
      float x = bullet_center.x;
      float y = img_height - t * (img_height - bullet_center.y);
      
      cv::Point2f pixel_point(x, y);
      
      // 检查点是否在图像范围内
      if (pixel_point.x >= 0 && pixel_point.y >= 0 && pixel_point.x < img_width && pixel_point.y < img_height) {
        trajectory_points.push_back(pixel_point);
      }
    }
    
    // 输出轨迹点数量
    tools::logger()->info("Calculated {} trajectory points (only one history point)", trajectory_points.size());
  } else {
    // 没有历史位置，返回空轨迹
    tools::logger()->info("No bullet history, calculated 0 trajectory points");
  }
  
  return trajectory_points;
}

// 获取或创建轨迹（确保只有一条有效轨迹）
BulletTrajectory& get_or_create_trajectory(int id, double fire_time, const Eigen::Vector3d& fire_direction, double bullet_v0, double current_time) {
  // 清理所有有效的轨迹（只保留一条）
  // 首先移除所有过期的轨迹
  bullet_trajectories.erase(
    std::remove_if(bullet_trajectories.begin(), bullet_trajectories.end(),
                  [current_time](const BulletTrajectory& traj) {
                    return current_time - traj.last_detected_time > 1.0 || traj.alpha <= 0;
                  }),
    bullet_trajectories.end());
  
  // 如果已经有轨迹，更新第一条轨迹
  if (!bullet_trajectories.empty()) {
    auto& trajectory = bullet_trajectories[0];
    trajectory.id = id;
    trajectory.fire_time = fire_time;
    trajectory.fire_direction = fire_direction;
    trajectory.bullet_v0 = bullet_v0;
    trajectory.last_detected_time = current_time;
    trajectory.alpha = 1.0;  // 重置透明度
    return trajectory;
  }
  
  // 创建新轨迹
  bullet_trajectories.emplace_back(id, fire_time, fire_direction, bullet_v0, current_time);
  return bullet_trajectories.back();
}

// 更新轨迹透明度和清理过期轨迹
void update_trajectories(double current_time) {
  // 遍历轨迹，更新透明度
  for (auto& trajectory : bullet_trajectories) {
    double time_since_detected = current_time - trajectory.last_detected_time;
    if (time_since_detected > 0) {
      // 逐渐降低透明度
      trajectory.alpha = 1.0 - time_since_detected;
      if (trajectory.alpha < 0) trajectory.alpha = 0;
    }
  }
  
  // 清理过期轨迹（透明度为0且超过1秒未检测到）
  bullet_trajectories.erase(
    std::remove_if(bullet_trajectories.begin(), bullet_trajectories.end(),
                  [current_time](const BulletTrajectory& traj) {
                    return current_time - traj.last_detected_time > 1.0 && traj.alpha <= 0;
                  }),
    bullet_trajectories.end());
  
  // 确保只有一条轨迹（如果有多条，只保留第一条）
  if (bullet_trajectories.size() > 1) {
    std::vector<BulletTrajectory> temp;
    temp.push_back(bullet_trajectories[0]);
    bullet_trajectories.swap(temp);
  }
}

// 绘制轨迹（使用平滑的弧线）
void draw_trajectory(cv::Mat& img, const BulletTrajectory& trajectory) {
  if (trajectory.trajectory_points.size() < 2) return;
  
  // 设置线条颜色
  int blue = 255;
  int green = 0;
  int red = 0;
  
  // 使用polylines绘制平滑的弧线
  std::vector<cv::Point> points;
  for (const auto& p : trajectory.trajectory_points) {
    points.push_back(cv::Point(static_cast<int>(p.x), static_cast<int>(p.y)));
  }
  
  // 绘制蓝色弧线，使用更平滑的线条
  cv::polylines(img, points, false, cv::Scalar(blue, green, red), 2, cv::LINE_AA);
  
  // 输出绘制信息
  tools::logger()->info("Drawing trajectory with {} points", trajectory.trajectory_points.size());
}

int main(int argc, char * argv[]) {
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");

  tools::Plotter plotter;
  tools::Exiter exiter;

  auto video_path = fmt::format("{}.avi", input_path);
  auto text_path = fmt::format("{}.txt", input_path);
  cv::VideoCapture video(video_path);
  std::ifstream text(text_path);

  auto_aim::YOLO yolo(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);

  // 初始化弹丸检测相关组件
  aimer::CoordConverter converter(config_path);
  aimer::aim::DoReproj do_reproj(
    converter.get_f_cv_mat_ref().clone(),
    converter.get_rot_ic_sup_cv_mat_ref().clone()
  );
  aimer::aim::DetectBullet bullet_detector(do_reproj);
  aimer::aim::AimCorrector aim_corrector(&converter);

  // 计算发射方向（由相机与枪管外参决定）
  Eigen::Vector3d fire_direction = Eigen::Vector3d(1.0, 0.0, 0.0);  // 默认相机正前方

  cv::Mat img, drawing;
  auto t0 = std::chrono::steady_clock::now();

  auto_aim::Target last_target;
  io::Command last_command;
  double last_t = -1;

  video.set(cv::CAP_PROP_POS_FRAMES, start_index);
  for (int i = 0; i < start_index; i++) {
    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
  }

  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;

    video.read(img);
    if (img.empty()) break;

    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
    auto timestamp = t0 + std::chrono::microseconds(int(t * 1e6));

    /// 自瞄核心逻辑

    solver.set_R_gimbal2world({w, x, y, z});

    auto yolo_start = std::chrono::steady_clock::now();
    auto armors = yolo.detect(img, frame_count);

    auto tracker_start = std::chrono::steady_clock::now();
    auto targets = tracker.track(armors, timestamp);

    auto aimer_start = std::chrono::steady_clock::now();
    auto command = aimer.aim(targets, timestamp, 27, false);

    if (
      !targets.empty() && aimer.debug_aim_point.valid &&
      std::abs(command.yaw - last_command.yaw) * 57.3 < 2)
      command.shoot = true;

    if (command.control) last_command = command;
    /// 调试输出

    auto finish = std::chrono::steady_clock::now();
    tools::logger()->info(
      "[{}] yolo: {:.1f}ms, tracker: {:.1f}ms, aimer: {:.1f}ms", frame_count,
      tools::delta_time(tracker_start, yolo_start) * 1e3,
      tools::delta_time(aimer_start, tracker_start) * 1e3,
      tools::delta_time(finish, aimer_start) * 1e3);

    tools::draw_text(
      img,
      fmt::format(
        "command is {},{:.2f},{:.2f},shoot:{}", command.control, command.yaw * 57.3,
        command.pitch * 57.3, command.shoot),
      {10, 60}, {154, 50, 205});

    Eigen::Quaternion gimbal_q = {w, x, y, z};
    tools::draw_text(
      img,
      fmt::format(
        "gimbal yaw{:.2f}", (tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0) * 57.3)[0]),
      {10, 90}, {255, 255, 255});

    nlohmann::json data;

    // 装甲板原始观测数据
    data["armor_num"] = armors.size();
    if (!armors.empty()) {
      const auto & armor = armors.front();
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
      data["armor_center_x"] = armor.center_norm.x;
      data["armor_center_y"] = armor.center_norm.y;
    }

    Eigen::Quaternion q{w, x, y, z};
    auto yaw = tools::eulers(q, 2, 1, 0)[0];
    data["gimbal_yaw"] = yaw * 57.3;
    data["cmd_yaw"] = command.yaw * 57.3;
    data["shoot"] = command.shoot;

    if (!targets.empty()) {
      auto target = targets.front();

      if (last_t == -1) {
        last_target = target;
        last_t = t;
        continue;
      }

      std::vector<Eigen::Vector4d> armor_xyza_list;

      // 当前帧target更新后
      armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      // aimer瞄准位置
      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid) tools::draw_points(img, image_points, {0, 0, 255});

      // 观测器内部数据
      Eigen::VectorXd x = target.ekf_x();
      data["x"] = x[0];
      data["vx"] = x[1];
      data["y"] = x[2];
      data["vy"] = x[3];
      data["z"] = x[4];
      data["vz"] = x[5];
      data["a"] = x[6] * 57.3;
      data["w"] = x[7];
      data["r"] = x[8];
      data["l"] = x[9];
      data["h"] = x[10];
      data["last_id"] = target.last_id;

      // 卡方检验数据
      data["residual_yaw"] = target.ekf().data.at("residual_yaw");
      data["residual_pitch"] = target.ekf().data.at("residual_pitch");
      data["residual_distance"] = target.ekf().data.at("residual_distance");
      data["residual_angle"] = target.ekf().data.at("residual_angle");
      data["nis"] = target.ekf().data.at("nis");
      data["nees"] = target.ekf().data.at("nees");
      data["nis_fail"] = target.ekf().data.at("nis_fail");
      data["nees_fail"] = target.ekf().data.at("nees_fail");
      data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");
    }

    /// 弹丸检测与匹配
    // 设置当前时间和图像
    converter.set_img_t(t);
    Eigen::Quaterniond q_gimbal = {w, x, y, z};
    converter.update(img.clone(), q_gimbal, t);
    
    // 检测弹丸
    std::vector<aimer::aim::ImageBullet> detected_bullets = 
      bullet_detector.process_new_frame(img.clone(), q_gimbal);
    
    // 处理检测到的弹丸
    if (!detected_bullets.empty()) {
      tools::logger()->info("[{}] Detected {} bullets", frame_count, detected_bullets.size());
      
      // 存储每个检测弹丸的信息和偏差
      struct BulletWithError {
        aimer::aim::ImageBullet bullet;
        double error;
        Eigen::Vector3d error_vec;
        int id;
      };
      std::vector<BulletWithError> bullets_with_error;
      
      // 对每个检测弹丸计算最佳发射时间并创建模拟弹丸
      for (int i = 0; i < detected_bullets.size(); i++) {
        aimer::aim::ImageBullet detected = detected_bullets[i];
        
        // 动态计算最佳发射时间
        double best_fire_time = find_best_fire_time(
          &converter, detected, fire_direction, BULLET_V0);
        
        // 创建模拟弹丸（使用唯一ID）
        int bullet_id = 999 + i;  // 不同弹丸使用不同ID
        create_simulated_bullet(aim_corrector, best_fire_time, fire_direction, 10.0, bullet_id, BULLET_V0);
        
        // 获取或创建轨迹
        BulletTrajectory& trajectory = get_or_create_trajectory(bullet_id, best_fire_time, fire_direction, BULLET_V0, t);
        
        // 添加当前弹丸位置到历史位置列表
        trajectory.bullet_history.push_back(detected.center);
        
        // 限制历史位置列表的大小，只保留最近的10个位置
        if (trajectory.bullet_history.size() > 10) {
          trajectory.bullet_history.erase(trajectory.bullet_history.begin());
        }
        
        // 计算轨迹点（基于历史位置）
        trajectory.trajectory_points = calculate_trajectory_points(converter, best_fire_time, t, fire_direction, BULLET_V0, trajectory.bullet_history);
        
        // 输出轨迹点数量
        tools::logger()->info("[{}] Trajectory {} has {} points, {} history points", frame_count, trajectory.id, trajectory.trajectory_points.size(), trajectory.bullet_history.size());
      }
      
      // 弹丸匹配和误差计算（只需要调用一次）
      aim_corrector.sample_aim_errors();
      
      // 获取预测的弹丸位置
      std::vector<aimer::aim::IdPos> predicted_bullets = aim_corrector.get_bullets();
      
      // 对每个检测弹丸计算与对应预测弹丸的偏差
      for (int i = 0; i < detected_bullets.size(); i++) {
        aimer::aim::ImageBullet detected = detected_bullets[i];
        int bullet_id = 999 + i;  // 与创建模拟弹丸时使用的ID相同
        
        // 计算当前检测弹丸与对应预测弹丸的偏差
        if (!predicted_bullets.empty()) {
          // 找到对应ID的预测弹丸
          for (const auto& pred : predicted_bullets) {
            if (pred.id == bullet_id) {
              // 计算偏差
              Eigen::Vector3d error_vec = calculate_bullet_error(
                  converter, detected, pred);
              double error = error_vec.norm();
              
              // 存储结果
              bullets_with_error.push_back({detected, error, error_vec, bullet_id});
              break;
            }
          }
        }
      }
      
      // 筛选偏差最小的弹丸
      if (!bullets_with_error.empty()) {
        // 按偏差排序
        std::sort(bullets_with_error.begin(), bullets_with_error.end(),
                  [](const BulletWithError& a, const BulletWithError& b) {
                      return a.error < b.error;
                  });
        
        // 获取最优弹丸
        BulletWithError best_bullet = bullets_with_error[0];
        
        // 输出最优弹丸的偏差
        tools::logger()->info(
          "[{}] Best bullet error: {:.3f} meters (ID: {}), Error vector: [{:.3f}, {:.3f}, {:.3f}]", 
          frame_count, best_bullet.error, best_bullet.id,
          best_bullet.error_vec.x(), best_bullet.error_vec.y(), best_bullet.error_vec.z());
        
        // 在图像上标记最优弹丸
        cv::circle(img, best_bullet.bullet.center, best_bullet.bullet.radius, cv::Scalar(0, 255, 0), 2);
        tools::draw_text(
          img,
          fmt::format("Error: {:.3f}", best_bullet.error),
          {static_cast<int>(best_bullet.bullet.center.x + 10), static_cast<int>(best_bullet.bullet.center.y - 10)},
          {0, 255, 0});
        
        // 存储弹丸数据
        data["bullet_detected"] = true;
        data["bullet_count"] = detected_bullets.size();
        data["best_bullet_error"] = best_bullet.error;
        data["best_bullet_error_x"] = best_bullet.error_vec.x();
        data["best_bullet_error_y"] = best_bullet.error_vec.y();
        data["best_bullet_error_z"] = best_bullet.error_vec.z();
      }
    } else {
      tools::logger()->info("[{}] No bullets detected", frame_count);
      data["bullet_detected"] = false;
      data["bullet_count"] = 0;
    }
    
    // 更新轨迹透明度和清理过期轨迹
    update_trajectories(t);
    
    // 绘制所有轨迹
    tools::logger()->info("[{}] Number of trajectories: {}", frame_count, bullet_trajectories.size());
    for (const auto& trajectory : bullet_trajectories) {
      if (trajectory.alpha > 0) {
        tools::logger()->info("[{}] Drawing trajectory {} with {} points and alpha {}", frame_count, trajectory.id, trajectory.trajectory_points.size(), trajectory.alpha);
        draw_trajectory(img, trajectory);
      }
    }

    plotter.plot(data);

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(30);
    if (key == 'q') break;
  }

  return 0;
}
