#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  std::mutex plan_mutex;
  auto_aim::Plan latest_plan = {};

  tools::ThreadSafeQueue<std::list<auto_aim::Target>, true> targets_queue(1);
  targets_queue.push({});

  std::atomic<bool> quit = false;
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      auto targets = targets_queue.front();
      auto gs = gimbal.state();
      
      auto target = targets.empty() ? std::nullopt : std::optional<auto_aim::Target>(targets.front());
      auto plan = planner.plan(target, gs.bullet_speed, gs.yaw, gs.pitch);

      io::Command command;
      command.control = plan.control;
      command.shoot = plan.fire;
      command.yaw = plan.yaw;
      command.pitch = plan.pitch;

      Eigen::Vector3d gimbal_pos(gs.yaw, gs.pitch, 0.0);

      bool shooter_fire = shooter.shoot(command, aimer, targets, gimbal_pos);

      bool final_fire = plan.fire && shooter_fire;

            // // 检查目标平移速度是否超过阈值
      // bool velocity_threshold_check = true;
      // if (target.has_value()) {
      //   // 获取目标平移速度向量
      //   double vx = target->ekf_x()[1];
      //   double vy = target->ekf_x()[3];
      //   double vz = target->ekf_x()[5];
        
      //   // 计算速度大小
      //   double velocity_magnitude = std::sqrt(vx*vx + vy*vy + vz*vz);
        
      //   // 设置速度阈值（硬编码）
      //   const double VELOCITY_THRESHOLD = 2.0; // 单位：m/s
        
      //   // 如果速度超过阈值，禁止开火
      //   if (velocity_magnitude > VELOCITY_THRESHOLD) {
      //     velocity_threshold_check = false;
      //   }
      // }

      // bool final_fire = plan.fire && shooter_fire && velocity_threshold_check;
      
      // 检查目标yaw偏航角速度是否超过阈值（允许平移但不允许自旋转）
      // bool yaw_velocity_threshold_check = true;
      // if (target.has_value()) {
      //   // 获取目标yaw角速度
      //   double v_yaw = target->ekf_x()[7];
        
      //   // 计算角速度大小（取绝对值）
      //   double yaw_velocity_magnitude = std::abs(v_yaw);
        
      //   // 设置yaw角速度阈值（硬编码）
      //   const double YAW_VELOCITY_THRESHOLD = 0.5; // 单位：rad/s
        
      //   // 如果yaw角速度超过阈值，禁止开火
      //   if (yaw_velocity_magnitude > YAW_VELOCITY_THRESHOLD) {
      //     yaw_velocity_threshold_check = false;
      //   }
      // }

      // bool final_fire = plan.fire && shooter_fire && yaw_velocity_threshold_check;

      {
        std::lock_guard<std::mutex> lock(plan_mutex);
        latest_plan = plan;
        latest_plan.fire = final_fire;
      }

      gimbal.send(
        plan.control, final_fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
        plan.pitch_acc);

      auto fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;

      nlohmann::json data;
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

      data["gimbal_yaw"] = gs.yaw;
      data["gimbal_yaw_vel"] = gs.yaw_vel;
      data["gimbal_pitch"] = gs.pitch;
      data["gimbal_pitch_vel"] = gs.pitch_vel;

      data["target_yaw"] = plan.target_yaw;
      data["target_pitch"] = plan.target_pitch;

      data["plan_yaw"] = plan.yaw;
      data["plan_yaw_vel"] = plan.yaw_vel;
      data["plan_yaw_acc"] = plan.yaw_acc;

      data["plan_pitch"] = plan.pitch;
      data["plan_pitch_vel"] = plan.pitch_vel;
      data["plan_pitch_acc"] = plan.pitch_acc;

      data["fire"] = final_fire ? 1 : 0;
      data["fired"] = fired ? 1 : 0;

      if (target.has_value()) {
        data["target_z"] = target->ekf_x()[4];   //z
        data["target_vz"] = target->ekf_x()[5];  //vz
      }

      if (target.has_value()) {
        data["w"] = target->ekf_x()[7];
      } else {
        data["w"] = 0.0;
      }

      plotter.plot(data);

      std::this_thread::sleep_for(10ms);
    }
  });

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    camera.read(img, t);
    auto q = gimbal.q(t);

    solver.set_R_gimbal2world(q);
    auto armors = yolo.detect(img);
    auto targets = tracker.track(armors, t);
    if (!targets.empty())
      targets_queue.push(targets);
    else
      targets_queue.push({});

    if (!targets.empty()) {
      auto target = targets.front();

      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      Eigen::Vector4d aim_xyza = planner.debug_xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      tools::draw_points(img, image_points, {0, 0, 255});
    }

    {
      std::lock_guard<std::mutex> lock(plan_mutex);
      auto text = fmt::format(
        "yaw: {:.2f} pitch: {:.2f} fire: {}", latest_plan.yaw, latest_plan.pitch, latest_plan.fire);
      cv::putText(img, text, {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);
    }

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}