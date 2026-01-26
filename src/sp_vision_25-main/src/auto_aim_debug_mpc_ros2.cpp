#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

// IO模块头文件
#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"

// 自动瞄准任务头文件
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"

// 工具模块头文件
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/ros2_debug.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{debug          |                        | 启用调试显示（需要 X11 显示环境）}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::ROS2DebugPublisher ros2_debug;

  // 解析命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  // 初始化硬件和算法模块
  io::Gimbal gimbal(config_path);      // 云台控制
  io::Camera camera(config_path);       // 相机

  auto_aim::YOLO yolo(config_path, false);  // 目标检测器
  auto_aim::Solver solver(config_path);       // PnP解算器
  auto_aim::Tracker tracker(config_path, solver);  // 目标跟踪器
  auto_aim::Planner planner(config_path);     // 运动规划器

  // 目标队列，用于在检测线程和规划线程间传递数据
  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  // 规划线程标志
  std::atomic<bool> quit = false;

  // 规划线程：负责目标预测和云台控制
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      // 获取当前目标
      auto target = target_queue.front();
      // 获取云台状态
      auto gs = gimbal.state();
      // 运动规划
      auto plan = planner.plan(target, gs.bullet_speed);

      // 发送控制指令到云台
      gimbal.send(
        plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
        plan.pitch_acc);

      // 检测是否发射
      auto fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;

      // 发布机器人姿态到ROS2
      ros2_debug.publishRobotPose(gs.yaw, gs.pitch);

      // 记录数据用于绘图
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

      data["fire"] = plan.fire ? 1 : 0;
      data["fired"] = fired ? 1 : 0;

      if (target.has_value()) {
        data["target_z"] = target->ekf_x()[4];   // z
        data["target_vz"] = target->ekf_x()[5];  // vz
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

  // 主检测线程
  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    // 读取图像
    camera.read(img, t);
    // 获取云台姿态
    auto q = gimbal.q(t);

    // 设置PnP解算器的旋转矩阵
    solver.set_R_gimbal2world(q);
    // 检测装甲板
    auto armors = yolo.detect(img);

    // 对每个装甲板进行PnP解算
    for (auto & armor : armors) {
      solver.solve(armor);
    }

    // 发布YOLO检测到的装甲板原始数据（经PnP解算后）
    ros2_debug.publishAutoAimArmors(armors);

    // 创建YOLO检测可视化图像（带绿色框）
    cv::Mat yolo_detection_img = img.clone();
    for (const auto & armor : armors) {
      tools::draw_points(yolo_detection_img, armor.points, {0, 255, 0});
    }
    // 发布YOLO检测图像
    ros2_debug.publishYOLODetectionImage(yolo_detection_img);

    // 跟踪目标
    auto targets = tracker.track(armors, t);

    // 更新目标队列（应该在这个位置更新加弹丸识别）
    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);

    // 发布tracker后的目标信息
    if (!targets.empty()) {
      auto target = targets.front();
      ros2_debug.publishTargetPose(target);
      ros2_debug.publishTargetInfo(target);
    } else {
      ros2_debug.publishTargetPose(std::nullopt);
      ros2_debug.publishTargetInfo(std::nullopt);
    }

    // 可视化：绘制装甲板角点和瞄准点
    if (!targets.empty()) {
      auto target = targets.front();

      // 绘制装甲板角点重投影（绿色）
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      // 绘制瞄准点（红色）
      Eigen::Vector4d aim_xyza = planner.debug_xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      tools::draw_points(img, image_points, {0, 0, 255});
    }

    // 发布tracker后的调试图像
    ros2_debug.publishDebugImage(img);

    // 显示图像
    // cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    // cv::imshow("reprojection", img);
    // auto key = cv::waitKey(1);
    // if (key == 'q') break;
  }

  // 清理：停止规划线程
  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  // 发送停止指令到云台
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  rclcpp::shutdown();

  return 0;
}
