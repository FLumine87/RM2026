#include <fmt/core.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <chrono>
#include <thread>
#include <iostream>
#include <vector>

#include "tools/recorder.hpp"
#include "tools/logger.hpp"
#include "tools/exiter.hpp"
#include "io/camera.hpp"

int main(int argc, char * argv[]) {
  std::cout << "=== Recorder Test ===" << std::endl;
  
  // 检查命令行参数
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <config_path>" << std::endl;
    return 1;
  }
  
  std::string config_path = argv[1];
  
  // 初始化 logger
  tools::logger()->info("Starting recorder test...");
  
  // 创建退出控制器（模拟主程序）
  tools::Exiter exiter;
  
  // 创建相机
  io::Camera camera(config_path);
  
  // 创建 recorder，与主程序相同配置
  tools::Recorder recorder(15, true);  // 15fps, 启用录制
  
  std::cout << "Recording... Press 'q' to quit." << std::endl;
  
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  int frame_count = 0;
  
  // 模拟主程序的主循环
  while (!exiter.exit()) {
    // 从相机读取图像
    camera.read(img, timestamp);
    
    // 检查图像是否有效
    if (img.empty()) {
      std::cerr << "Error: Empty frame from camera!" << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    
    // 获取云台姿态（模拟）
    Eigen::Quaterniond q(1, 0, 0, 0);  // 单位四元数
    
    // 录制帧
    recorder.record(img.clone(), q, timestamp);
    
    // 在图像上绘制帧号
    frame_count++;
    std::string text = fmt::format("Frame: {}, FPS: 15", frame_count);
    cv::putText(img, text, cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 
                1.0, cv::Scalar(0, 255, 0), 2);
    
    // 显示图像（模拟主程序）
    cv::Mat display_img;
    cv::resize(img, display_img, {}, 0.5, 0.5);
    cv::imshow("Recorder Test", display_img);
    
    // 键盘事件处理（模拟主程序）
    auto key = cv::waitKey(1);
    if (key == 'q') {
      std::cout << "User pressed 'q', quitting..." << std::endl;
      break;
    }
    
    // 进度输出
    if (frame_count % 100 == 0) {
      std::cout << "Recorded " << frame_count << " frames" << std::endl;
      tools::logger()->info("Recorded {} frames", frame_count);
    }
  }
  
  std::cout << "Test completed! Recorded " << frame_count << " frames." << std::endl;
  std::cout << "Check 'records' folder for output." << std::endl;
  tools::logger()->info("Test completed! Recorded {} frames.", frame_count);
  
  // 等待录制线程完成
  std::this_thread::sleep_for(std::chrono::seconds(2));
  
  cv::destroyAllWindows();
  return 0;
}
