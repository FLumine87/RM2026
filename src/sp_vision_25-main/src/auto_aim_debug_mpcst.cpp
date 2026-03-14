#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/trajectory.hpp"

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
      auto plan = planner.plan(target, gs.bullet_speed, gs);

      auto timestamp = std::chrono::steady_clock::now();
      auto aimer_command = aimer.aim_with_plan(targets, plan, timestamp, gs.bullet_speed);

      // 计算基于旋转中心的线性预测 yaw
      double send_yaw = plan.yaw;
      if (true) {
        // 新方案：基于旋转中心的线性预测
        double fly_time = 0.223;  // 默认飞行时间
        if (target.has_value()) {
          // 从 EKF 获取旋转中心位置和速度
          double center_x = target->ekf_x()[0];
          double center_y = target->ekf_x()[2];
          double center_z = target->ekf_x()[4];
          double ekf_yaw = target->ekf_x()[6];    // 旋转中心当前 yaw
          double ekf_yaw_v = target->ekf_x()[7];  // 旋转中心角速度

          // 计算距离并使用 tools 计算飞行时间
          double dist = std::sqrt(center_x * center_x + center_y * center_y);
          tools::Trajectory bullet_traj(gs.bullet_speed, dist, center_z);
          if (!bullet_traj.unsolvable) {
            fly_time = bullet_traj.fly_time;
          }

          // 线性预测：yaw + fly_time * yaw_v
          send_yaw = tools::limit_rad(ekf_yaw + fly_time * ekf_yaw_v);
        }
      } else {
        // 旧方案：使用 plan.yaw
        send_yaw = plan.yaw;
      }

      io::Command command;
      command.control = plan.control;
      command.fire = plan.fire;
      command.yaw = send_yaw;  // 使用实际发送的 yaw
      command.pitch = plan.pitch;

      Eigen::Vector3d gimbal_pos(gs.yaw, gs.pitch, 0.0);

      bool shooter_fire = shooter.shoot(command, aimer, targets, gimbal_pos);

      bool final_fire = plan.fire && shooter_fire;

      {
        std::lock_guard<std::mutex> lock(plan_mutex);
        latest_plan = plan;
        latest_plan.fire = final_fire;
      }

      gimbal.send(
        plan.control, final_fire, send_yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
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
    targets_queue.push(targets);

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