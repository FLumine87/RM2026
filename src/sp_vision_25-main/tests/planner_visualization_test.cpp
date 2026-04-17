#include "tasks/auto_aim/planner/planner.hpp"

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |     | 输出命令行参数说明    }"
  "{d              | 3.0 | Target距离(m)       }"
  "{w              | 5.0 | Target角速度(rad/s) }"
  "{@config-path   |     | yaml配置文件路径     }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  auto d = cli.get<double>("d");
  auto w = cli.get<double>("w");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  auto_aim::Planner planner(config_path);
  auto_aim::Target target(d, w, 0.2, 0.2);

  auto x = target.ekf_x();
//  x[0] = 0.0; // x
  x[1] = 0.0; // vx
  x[2] = 0.0; // y
  x[3] = 0.4; // vy
  x[4] = 0.0; // z
  x[5] = 0.0; // vz
  x[6] = 0.0; // yaw
//  x[7] = 0.0; // vyaw
//  x[8] = 0.0; // 短轴长
  x[9] = 0.1; // 长轴差值
//  x[10] = 0.1; // 装甲板高低差

  // 生成轨迹数据
  std::vector<double> time_points;
  std::vector<double> target_yaw_list;
  std::vector<double> plan_yaw_list;

  // 生成1秒的轨迹数据，采样时间0.01秒
  for (double t = 0; t <= 1.0; t += 0.01) {
    time_points.push_back(t);
    
    // 预测目标状态
    target.predict(0.01);
    
    // 生成规划
    auto plan = planner.plan(target, 22.0);
    
    // 保存数据
    target_yaw_list.push_back(plan.target_yaw);
    plan_yaw_list.push_back(plan.yaw);
  }

  // 创建图像
  int width = 800;
  int height = 600;
  cv::Mat image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

  // 绘制坐标轴
  int margin = 50;
  cv::line(image, cv::Point(margin, margin), cv::Point(margin, height - margin), cv::Scalar(0, 0, 0), 2);
  cv::line(image, cv::Point(margin, height - margin), cv::Point(width - margin, height - margin), cv::Scalar(0, 0, 0), 2);

  // 计算数据范围
  double min_yaw = *std::min_element(target_yaw_list.begin(), target_yaw_list.end());
  double max_yaw = *std::max_element(target_yaw_list.begin(), target_yaw_list.end());
  double yaw_range = max_yaw - min_yaw;
  if (yaw_range < 0.01) yaw_range = 0.01;

  // 绘制目标轨迹（蓝色）
  for (size_t i = 1; i < target_yaw_list.size(); i++) {
    double t1 = time_points[i-1];
    double t2 = time_points[i];
    double y1 = target_yaw_list[i-1];
    double y2 = target_yaw_list[i];

    int x1 = margin + (t1 / 1.0) * (width - 2 * margin);
    int x2 = margin + (t2 / 1.0) * (width - 2 * margin);
    int y1_pixel = height - margin - ((y1 - min_yaw) / yaw_range) * (height - 2 * margin);
    int y2_pixel = height - margin - ((y2 - min_yaw) / yaw_range) * (height - 2 * margin);

    cv::line(image, cv::Point(x1, y1_pixel), cv::Point(x2, y2_pixel), cv::Scalar(255, 0, 0), 2);
  }

  // 绘制规划轨迹（红色）
  for (size_t i = 1; i < plan_yaw_list.size(); i++) {
    double t1 = time_points[i-1];
    double t2 = time_points[i];
    double y1 = plan_yaw_list[i-1];
    double y2 = plan_yaw_list[i];

    int x1 = margin + (t1 / 1.0) * (width - 2 * margin);
    int x2 = margin + (t2 / 1.0) * (width - 2 * margin);
    int y1_pixel = height - margin - ((y1 - min_yaw) / yaw_range) * (height - 2 * margin);
    int y2_pixel = height - margin - ((y2 - min_yaw) / yaw_range) * (height - 2 * margin);

    cv::line(image, cv::Point(x1, y1_pixel), cv::Point(x2, y2_pixel), cv::Scalar(0, 0, 255), 2);
  }

  // 添加标签
  cv::putText(image, "Time (s)", cv::Point(width/2 - 30, height - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
  cv::putText(image, "Yaw (rad)", cv::Point(10, height/2), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, cv::LINE_AA, true);
  cv::putText(image, "Target Yaw", cv::Point(width - 120, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0));
  cv::putText(image, "Plan Yaw", cv::Point(width - 120, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));

  // 显示图像
  cv::imshow("MPC Planner Visualization", image);
  cv::waitKey(0);

  return 0;
}
