#include "io/gimbal/gimbal.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | configs/standard5.yaml | yaml配置文件路径}"
  "{record r       | false | 是否记录数据到文件}"
  "{verbose v      | false | 是否显示详细输出}"
  "{visualize z    | false | 是否显示可视化界面}"
  "{rate           | 20 | 刷新频率(Hz)}";

int main(int argc, char *argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  auto record = cli.get<bool>("record");
  auto verbose = cli.get<bool>("verbose");
  auto visualize = cli.get<bool>("visualize");
  auto rate = cli.get<int>("rate");

  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  io::Gimbal gimbal(config_path);

  tools::logger()->info("Gimbal serial standalone started.");
  tools::logger()->info("Press Ctrl+C to exit");

  // 1. 数据记录功能
  std::ofstream log_file;
  if (record) {
    // log_file.open("gimbal_data.csv");
    // log_file << "time,mode,yaw,yaw_vel,pitch,pitch_vel,bullet_speed,bullet_count\n";
    // tools::logger()->info("Data recording enabled. Saving to gimbal_data.csv");
  }

  // 2. 可视化增强（可选）
  cv::Mat display(400, 800, CV_8UC3, cv::Scalar(20, 20, 20));
  if (visualize) {
    cv::namedWindow("Gimbal Status", cv::WINDOW_NORMAL);
    tools::logger()->info("Visualization enabled");
  }

  auto loop_interval = std::chrono::milliseconds(1000 / rate);
  uint16_t last_bullet_count = 0;
  io::GimbalMode last_mode = gimbal.mode();

  while (!exiter.exit()) {
    auto t = std::chrono::steady_clock::now();
    auto state = gimbal.state();
    auto mode = gimbal.mode();
    auto q = gimbal.q(t);
    auto ypr = tools::eulers(q, 2, 1, 0);

    // 检测模式变化
    if (mode != last_mode) {
      tools::logger()->info("Gimbal mode changed: {}", gimbal.str(mode));
      last_mode = mode;
    }

    // 检测开火事件
    auto fired = state.bullet_count > last_bullet_count;
    if (fired) {
      tools::logger()->info("Gimbal fired! Current count: {}", state.bullet_count);
    }
    last_bullet_count = state.bullet_count;

    // 3. 命令行交互模式 - 简洁输出
    if (verbose) {
      // 详细输出
      tools::logger()->info(
        "Mode: {} | Yaw: {:.2f}° ({:.2f}°/s) | Pitch: {:.2f}° ({:.2f}°/s) | "
        "Bullet: {:.1f}m/s [#{}]",
        gimbal.str(mode),
        state.yaw, state.yaw_vel,
        state.pitch, state.pitch_vel,
        state.bullet_speed, state.bullet_count
      );
    } else {
      // 简洁单行输出
      std::printf("\rMode: %s | Yaw: %6.2f° | Pitch: %6.2f° | Bullet: %3.0f m/s [#%d]    ",
        gimbal.str(mode).c_str(),
        state.yaw, state.pitch,
        state.bullet_speed, state.bullet_count
      );
      std::fflush(stdout);
    }

    // 4. 数据记录
    if (record && log_file.is_open()) {
      auto elapsed = std::chrono::duration<double>(t.time_since_epoch()).count();
      log_file << elapsed << ","
               << static_cast<int>(mode) << ","
               << state.yaw << ","
               << state.yaw_vel << ","
               << state.pitch << ","
               << state.pitch_vel << ","
               << state.bullet_speed << ","
               << state.bullet_count << "\n";
    }

    // 5. 可视化界面
    if (visualize) {
      display = cv::Scalar(20, 20, 20);
      
      // 绘制状态信息
      int y = 30;
      int line_height = 25;
      
      cv::putText(display, "Gimbal Serial Monitor", cv::Point(20, y),
                 cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
      y += line_height * 2;
      
      cv::putText(display, "Mode: " + gimbal.str(mode), cv::Point(20, y),
                 cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
      y += line_height;
      
      cv::putText(display, "Yaw: " + std::to_string(state.yaw) + "°", cv::Point(20, y),
                 cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
      y += line_height;
      
      cv::putText(display, "Pitch: " + std::to_string(state.pitch) + "°", cv::Point(20, y),
                 cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
      y += line_height;
      
      cv::putText(display, "Bullet Speed: " + std::to_string(state.bullet_speed) + " m/s", cv::Point(20, y),
                 cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
      y += line_height;
      
      cv::putText(display, "Bullet Count: " + std::to_string(state.bullet_count), cv::Point(20, y),
                 cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
      y += line_height * 2;
      
      cv::putText(display, "Press Ctrl+C to exit", cv::Point(20, y),
                 cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 1);
      
      cv::imshow("Gimbal Status", display);
      cv::waitKey(1);
    }

    // 以下是打印云台状态的功能（处于注释状态）
    /*
    // 详细打印云台状态
    tools::logger()->info("=== Gimbal Status ===");
    tools::logger()->info("Mode: {}", gimbal.str(mode));
    tools::logger()->info("Yaw: {:.2f}° (Velocity: {:.2f}°/s)", state.yaw, state.yaw_vel);
    tools::logger()->info("Pitch: {:.2f}° (Velocity: {:.2f}°/s)", state.pitch, state.pitch_vel);
    tools::logger()->info("Bullet Speed: {:.1f} m/s", state.bullet_speed);
    tools::logger()->info("Bullet Count: {}", state.bullet_count);
    tools::logger()->info("Quaternion: w={:.3f}, x={:.3f}, y={:.3f}, z={:.3f}",
                        q.w(), q.x(), q.y(), q.z());
    tools::logger()->info("Euler Angles: yaw={:.2f}°, pitch={:.2f}°, roll={:.2f}°",
                        ypr[0] * 57.3, ypr[1] * 57.3, ypr[2] * 57.3);
    tools::logger()->info("====================");
    */

    std::this_thread::sleep_for(loop_interval);
  }

  // 清理资源
  if (record && log_file.is_open()) {
    log_file.close();
    tools::logger()->info("Data saved to gimbal_data.csv");
  }

  if (visualize) {
    cv::destroyAllWindows();
  }

  tools::logger()->info("Gimbal serial standalone stopped.");
  return 0;
}
