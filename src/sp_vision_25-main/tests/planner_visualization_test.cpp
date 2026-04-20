#include "tasks/auto_aim/planner/planner.hpp"

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |     | 输出命令行参数说明    }"
  "{d              | 3.0 | Target距离(m)       }"
  "{w              | 15.0 | Target角速度(rad/s) }"
  "{vx             | 0.0 | Target x方向速度(m/s) }"
  "{vy             | 0.0 | Target y方向速度(m/s) }"
  "{vz             | 0.0 | Target z方向速度(m/s) }"
  "{@config-path   |     | yaml配置文件路径     }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  auto d = cli.get<double>("d");
  auto w = cli.get<double>("w");
  auto vx = cli.get<double>("vx");
  auto vy = cli.get<double>("vy");
  auto vz = cli.get<double>("vz");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  auto_aim::Planner planner(config_path);
  auto_aim::Target target(d, w, 0.2, 0.2);

  // 自定义 EKF 结果，完整映射目标状态
  auto x = target.ekf_x();
  x[0] = d;      // x 位置
  x[1] = vx;     // x 方向速度
  x[2] = 0.0;    // y 位置
  x[3] = vy;     // y 方向速度
  x[4] = 0.0;    // z 位置
  x[5] = vz;     // z 方向速度
  x[6] = 0.0;    // yaw 角度
  x[7] = w;      // yaw 角速度
  x[8] = 0.4;    // 短轴长
  x[9] = 0.0;    // 长轴差值
  x[10] = 0.0;   // 装甲板高低差

  target.set_ekf_x(x);

  // 生成轨迹数据
  std::vector<double> time_points;
  std::vector<double> target_yaw_list;
  std::vector<double> plan_yaw_list;
  std::vector<double> target_pitch_list;
  std::vector<double> plan_pitch_list;
  std::vector<bool> fireable_list;

  // 生成1秒的轨迹数据，采样时间0.01秒
  for (double t = 0; t <= 1.0; t += 0.01) {
    time_points.push_back(t);

    // 保存初始状态的副本，确保每次规划使用同一帧的状态
    auto_aim::Target target_copy = target;
    
    // 预测目标状态到当前时间点
    target_copy.predict(t);

    // 生成规划（使用无收敛判断的接口）
    auto plan = planner.plan(target_copy, 22.0);

    // 保存数据
    target_yaw_list.push_back(plan.target_yaw);
    plan_yaw_list.push_back(plan.yaw);
    target_pitch_list.push_back(plan.target_pitch);
    plan_pitch_list.push_back(plan.pitch);
    fireable_list.push_back(plan.fire);
  }

  int width = 800;
  int height = 1000;
  cv::Mat image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

  // 绘制坐标轴
  int margin = 50;
  int half_height = height / 2;

  cv::line(image, cv::Point(margin, margin), cv::Point(margin, half_height - margin), cv::Scalar(0, 0, 0), 2);
  cv::line(image, cv::Point(margin, half_height - margin), cv::Point(width - margin, half_height - margin), cv::Scalar(0, 0, 0), 2);
  cv::line(image, cv::Point(margin, half_height + margin), cv::Point(margin, height - margin), cv::Scalar(0, 0, 0), 2);
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
    int y1_pixel = half_height - margin - ((y1 - min_yaw) / yaw_range) * (half_height - 2 * margin);
    int y2_pixel = half_height - margin - ((y2 - min_yaw) / yaw_range) * (half_height - 2 * margin);

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
    int y1_pixel = half_height - margin - ((y1 - min_yaw) / yaw_range) * (half_height - 2 * margin);
    int y2_pixel = half_height - margin - ((y2 - min_yaw) / yaw_range) * (half_height - 2 * margin);

    cv::line(image, cv::Point(x1, y1_pixel), cv::Point(x2, y2_pixel), cv::Scalar(0, 0, 255), 2);
  }

  for (size_t i = 0; i < plan_yaw_list.size(); i++) {
    if (fireable_list[i]) {
      int x_pixel = margin + (time_points[i] / 1.0) * (width - 2 * margin);
      int y_pixel = half_height - margin - ((plan_yaw_list[i] - min_yaw) / yaw_range) * (half_height - 2 * margin);
      cv::circle(image, cv::Point(x_pixel, y_pixel), 5, cv::Scalar(0, 255, 0), -1);
    }
  }

  double min_pitch = *std::min_element(target_pitch_list.begin(), target_pitch_list.end());
  double max_pitch = *std::max_element(target_pitch_list.begin(), target_pitch_list.end());
  double pitch_range = max_pitch - min_pitch;
  if (pitch_range < 0.01) pitch_range = 0.01;

  for (size_t i = 1; i < target_pitch_list.size(); i++) {
    double t1 = time_points[i-1];
    double t2 = time_points[i];
    double y1 = target_pitch_list[i-1];
    double y2 = target_pitch_list[i];

    int x1 = margin + (t1 / 1.0) * (width - 2 * margin);
    int x2 = margin + (t2 / 1.0) * (width - 2 * margin);
    int y1_pixel = height - margin - ((y1 - min_pitch) / pitch_range) * (half_height - 2 * margin);
    int y2_pixel = height - margin - ((y2 - min_pitch) / pitch_range) * (half_height - 2 * margin);

    cv::line(image, cv::Point(x1, y1_pixel), cv::Point(x2, y2_pixel), cv::Scalar(255, 0, 0), 2);
  }

  for (size_t i = 1; i < plan_pitch_list.size(); i++) {
    double t1 = time_points[i-1];
    double t2 = time_points[i];
    double y1 = plan_pitch_list[i-1];
    double y2 = plan_pitch_list[i];

    int x1 = margin + (t1 / 1.0) * (width - 2 * margin);
    int x2 = margin + (t2 / 1.0) * (width - 2 * margin);
    int y1_pixel = height - margin - ((y1 - min_pitch) / pitch_range) * (half_height - 2 * margin);
    int y2_pixel = height - margin - ((y2 - min_pitch) / pitch_range) * (half_height - 2 * margin);

    cv::line(image, cv::Point(x1, y1_pixel), cv::Point(x2, y2_pixel), cv::Scalar(0, 0, 255), 2);
  }

  for (size_t i = 0; i < plan_pitch_list.size(); i++) {
    if (fireable_list[i]) {
      int x_pixel = margin + (time_points[i] / 1.0) * (width - 2 * margin);
      int y_pixel = height - margin - ((plan_pitch_list[i] - min_pitch) / pitch_range) * (half_height - 2 * margin);
      cv::circle(image, cv::Point(x_pixel, y_pixel), 5, cv::Scalar(0, 255, 0), -1);
    }
  }

  cv::putText(image, "Time (s)", cv::Point(width/2 - 30, height - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
  cv::putText(image, "Yaw (rad)", cv::Point(10, half_height/2), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, cv::LINE_AA, true);
  cv::putText(image, "Pitch (rad)", cv::Point(10, half_height + half_height/2), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, cv::LINE_AA, true);
  cv::putText(image, "Target Yaw", cv::Point(width - 120, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0));
  cv::putText(image, "Plan Yaw", cv::Point(width - 120, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
  cv::putText(image, "Fire Point", cv::Point(width - 120, 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));

  // 显示图像
  cv::imshow("MPC Planner Visualization", image);
  cv::waitKey(0);

  return 0;
}
