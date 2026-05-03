#include "tasks/auto_aim/yolo.hpp"
#include "io/camera.hpp"

#include <opencv2/opencv.hpp>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{config-path c  | configs/camera.yaml | yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;

  auto config_path = cli.get<std::string>("config-path");
  io::Camera camera(config_path);
  auto_aim::YOLO yolo(config_path, true);

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  while (!exiter.exit()) {
    camera.read(img, timestamp);

    auto armors = yolo.detect(img);

    // 绘制检测结果
    for (const auto & armor : armors) {
      // 绘制装甲板边界框
      cv::rectangle(img, armor.box, cv::Scalar(0, 255, 0), 2);
      // 绘制中心点
      cv::circle(img, armor.center, 3, cv::Scalar(0, 0, 255), -1);
    }

    cv::imshow("YOLO Door Detection", img);
    if (cv::waitKey(1) == 'q') break;
  }

  return 0;
}