#include <fmt/core.h>

#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "serial/serial.h"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/crc.hpp"

#include <cmath>

const std::string keys =
  "{help h usage ?  |                          | 输出命令行参数说明}"
  "{@config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{serial-port p   | /dev/gimbal              | 串口端口(如 /dev/gimbal) }"
  "{baud b          | 115200                   | 串口波特率        }"
  "{output-folder o |      assets/img_with_q   | 输出文件夹路径   }";

void write_q(const std::string q_path, const Eigen::Quaterniond & q)
{
  std::ofstream q_file(q_path);
  Eigen::Vector4d wxyz = q.coeffs();
  // 输出顺序为wxyz
  q_file << fmt::format("{} {} {} {}", wxyz[0], wxyz[1], wxyz[2], wxyz[3]);
  q_file.close();
}

bool read_gimbal_packet(serial::Serial & ser, io::GimbalToVision & packet)
{
  if (!ser.isOpen()) return false;

  uint8_t byte = 0;
  while (ser.available() > 0) {
    if (ser.read(&byte, 1) != 1) return false;
    if (byte != 'S') continue;

    if (ser.read(&byte, 1) != 1) return false;
    if (byte != 'P') continue;

    packet.head[0] = 'S';
    packet.head[1] = 'P';

    auto payload_size = sizeof(packet) - sizeof(packet.head);
    if (ser.read(reinterpret_cast<uint8_t *>(&packet) + sizeof(packet.head), payload_size) !=
        payload_size)
      return false;

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&packet), sizeof(packet))) return false;

    return true;
  }

  return false;
}

void capture_loop(
  const std::string & config_path, const std::string & serial_port, uint32_t baud,
  const std::string & output_folder)
{
  serial::Serial ser(serial_port, baud, serial::Timeout::simpleTimeout(50));
  if (!ser.isOpen()) ser.open();
  io::Camera camera(config_path);
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  Eigen::Quaterniond last_q(1, 0, 0, 0);
  io::GimbalToVision packet;

  int count = 0;
  while (true) {
    camera.read(img, timestamp);
    Eigen::Quaterniond q = last_q;
    if (read_gimbal_packet(ser, packet)) {
      q = Eigen::Quaterniond(packet.q[0], packet.q[1], packet.q[2], packet.q[3]);
      last_q = q;
    }

    double w = packet.q[0];
    double x = packet.q[1];
    double y = packet.q[2];
    double z = packet.q[3];


    // tools::draw_text(img_with_ypr, fmt::format("Z_rad {:.4f}", ypr[0]), {40, 200}, {0, 0, 255});
    // tools::draw_text(img_with_ypr, fmt::format("Y_rad {:.4f}", ypr[1]), {40, 240}, {0, 0, 255});
    // tools::draw_text(img_with_ypr, fmt::format("X_rad {:.4f}", ypr[2]), {40, 280}, {0, 0, 255});

    // 在图像上显示欧拉角：Z/Y 使用串口包的 yaw/pitch（packet.yaw/pitch）*57.3，
    // X（roll）仍由四元数 q 计算得到
    auto img_with_ypr = img.clone();
    Eigen::Vector3d ypr = tools::eulers(q, 2, 1, 0);
    double yaw_deg = ypr[0] * 180.0 / M_PI;
    double pitch_deg = ypr[1] * 180.0 / M_PI;
    double roll_deg = ypr[2] * 180.0 / M_PI;

    tools::draw_text(img_with_ypr, fmt::format("yaw {:.2f}", yaw_deg), {40, 40}, {0, 0, 255});
    tools::draw_text(img_with_ypr, fmt::format("pitch {:.2f}", pitch_deg), {40, 80}, {0, 0, 255});
    tools::draw_text(img_with_ypr, fmt::format("roll {:.2f}", roll_deg), {40, 120}, {0, 0, 255});

    tools::draw_text(img_with_ypr, fmt::format("yaw_rad {:.4f}", ypr[0]), {40, 200}, {0, 0, 255});
    tools::draw_text(img_with_ypr, fmt::format("pitch_rad {:.4f}", ypr[1]), {40, 240}, {0, 0, 255});
    tools::draw_text(img_with_ypr, fmt::format("roll_rad {:.4f}", ypr[2]), {40, 280}, {0, 0, 255});

    std::vector<cv::Point2f> centers_2d;
    // auto success = cv::findCirclesGrid(img, cv::Size(10, 7), centers_2d);  // 默认是对称圆点图案
    // cv::drawChessboardCorners(img_with_ypr, cv::Size(10, 7), centers_2d, success);  // 显示识别结果
    auto success = cv::findCirclesGrid(img, cv::Size(7, 7), centers_2d);  // 默认是对称圆点图案
    cv::drawChessboardCorners(img_with_ypr, cv::Size(7, 7), centers_2d, success);  // 显示识别结果
    cv::resize(img_with_ypr, img_with_ypr, {}, 0.5, 0.5);  // 显示时缩小图片尺寸

    // 按“s”保存图片和对应四元数，按“q”退出程序
    cv::imshow("Press s to save, q to quit", img_with_ypr);
    auto key = cv::waitKey(1);
    if (key == 'q')
      break;
    else if (key != 's')
      continue;

    // 保存图片和四元数
    count++;
    auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
    auto q_path = fmt::format("{}/{}.txt", output_folder, count);
    cv::imwrite(img_path, img);
    write_q(q_path, q);
    tools::logger()->info("[{}] Saved in {}", count, output_folder);
  }

  // 离开该作用域时，camera和串口会自动关闭
}

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto serial_port = cli.get<std::string>("serial-port");
  auto baud = static_cast<uint32_t>(cli.get<int>("baud"));
  auto output_folder = cli.get<std::string>("output-folder");

  // 新建输出文件夹
  std::filesystem::create_directory(output_folder);

  tools::logger()->info("默认标定板尺寸为7列7行");
  // 主循环，保存图片和对应四元数
  capture_loop(config_path, serial_port, baud, output_folder);

  tools::logger()->warn("注意四元数输出顺序为wxyz");

  return 0;
}
