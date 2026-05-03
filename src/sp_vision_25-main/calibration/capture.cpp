#include <fmt/core.h>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include "io/camera.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

const std::string keys =
    "{help h usage ?   |                           | 输出命令行参数说明}"
    "{@config-path c   | configs/calibration.yaml  | yaml配置文件路径 }"
    "{output-folder o  | assets/img_with_q   | 输出图片文件夹路径 }";

void capture_loop(const std::string & config_path, const std::string & output_folder)
{
    // 初始化相机
    io::Camera camera(config_path);
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;

    int count = 0;
    tools::logger()->info("相机启动成功，开始图像采集...");

    while (true) {
        // 修改点：直接调用 read，因为该函数返回 void
        camera.read(img, timestamp);

        // 如果读取到的图像为空则跳过（防止相机初始化瞬间的空帧）
        if (img.empty()) {
            continue;
        }

        auto display_img = img.clone();

        // 实时识别标定板（7列 7行）
        std::vector<cv::Point2f> centers_2d;
        bool success = cv::findCirclesGrid(display_img, cv::Size(7, 7), centers_2d);
        
        // 绘制识别结果用于反馈
        cv::drawChessboardCorners(display_img, cv::Size(7, 7), centers_2d, success);

        // 预览图显示提示
        tools::draw_text(display_img, fmt::format("Saved: {}", count), {40, 40}, {0, 255, 0});
        tools::draw_text(display_img, "Press 's' to save, 'q' to quit", {40, 80}, {0, 255, 0});

        cv::imshow("Calibration Capture", display_img);
        
        auto key = cv::waitKey(1);
        if (key == 'q') {
            break;
        } else if (key == 's') {
            // 保存原始图
            count++;
            auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
            cv::imwrite(img_path, img);
            tools::logger()->info("[{}] 已保存图片: {}", count, img_path);
        }
    }
}

int main(int argc, char * argv[])
{
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
        cli.printMessage();
        return 0;
    }

    auto config_path = cli.get<std::string>(0);
    auto output_folder = cli.get<std::string>("output-folder");

    std::filesystem::create_directories(output_folder);

    tools::logger()->info("当前使用标定板: 10列 7行 圆点阵列");
    
    capture_loop(config_path, output_folder);

    return 0;
}