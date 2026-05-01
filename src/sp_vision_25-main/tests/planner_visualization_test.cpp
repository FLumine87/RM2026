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

  auto debug = planner.debug(target, 22.0);

  // 生成时间点（debug函数返回从-10ms到500ms，10ms步长）
  std::vector<double> time_points;
  for (int t_ms = -10; t_ms <= 500; t_ms += 10) {
    time_points.push_back(t_ms * 0.001);
  }
  int num_points = time_points.size();

  int width = 1200;
  int height = 900;
  cv::Mat image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

  int margin = 80;
  int graph_height = (height - 3 * margin) / 2;

  // 上半部分：Yaw
  int yaw_top = margin;
  int yaw_bottom = yaw_top + graph_height;
  int pitch_top = yaw_bottom + margin;
  int pitch_bottom = pitch_top + graph_height;

  cv::line(image, cv::Point(margin, yaw_bottom), cv::Point(width - margin, yaw_bottom), cv::Scalar(0, 0, 0), 2);
  cv::line(image, cv::Point(margin, pitch_bottom), cv::Point(width - margin, pitch_bottom), cv::Scalar(0, 0, 0), 2);
  cv::line(image, cv::Point(margin, yaw_top), cv::Point(margin, yaw_bottom), cv::Scalar(0, 0, 0), 2);
  cv::line(image, cv::Point(margin, pitch_top), cv::Point(margin, pitch_bottom), cv::Scalar(0, 0, 0), 2);

  for (int t_ms = 0; t_ms <= 500; t_ms += 100) {
    double t = t_ms * 0.001;
    int x_pos = margin + static_cast<int>((t + 0.01) / 0.51 * (width - 2 * margin));
    cv::line(image, cv::Point(x_pos, yaw_bottom - 5), cv::Point(x_pos, yaw_bottom + 5), cv::Scalar(0, 0, 0), 1);
    cv::line(image, cv::Point(x_pos, pitch_bottom - 5), cv::Point(x_pos, pitch_bottom + 5), cv::Scalar(0, 0, 0), 1);
    char buf[32];
    sprintf(buf, "%.1f", t);
    cv::putText(image, buf, cv::Point(x_pos - 10, yaw_bottom + 20), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0));
  }

  // ========== Yaw ==========
  double min_yaw = *std::min_element(debug.target_yaw_list.begin(), debug.target_yaw_list.end());
  double max_yaw = *std::max_element(debug.target_yaw_list.begin(), debug.target_yaw_list.end());
  double yaw_range = max_yaw - min_yaw;
  if (yaw_range < 0.01) yaw_range = 0.01;
  min_yaw -= yaw_range * 0.1;
  max_yaw += yaw_range * 0.1;
  yaw_range = max_yaw - min_yaw;

  // 绘制目标Yaw（红色）
  for (int i = 1; i < num_points; i++) {
    double t1 = time_points[i-1];
    double t2 = time_points[i];
    double y1 = debug.target_yaw_list[i-1];
    double y2 = debug.target_yaw_list[i];

    int x1 = margin + static_cast<int>((t1 + 0.01) / 0.51 * (width - 2 * margin));
    int x2 = margin + static_cast<int>((t2 + 0.01) / 0.51 * (width - 2 * margin));
    int y1_pixel = yaw_bottom - static_cast<int>(((y1 - min_yaw) / yaw_range) * (graph_height - 20));
    int y2_pixel = yaw_bottom - static_cast<int>(((y2 - min_yaw) / yaw_range) * (graph_height - 20));

    cv::line(image, cv::Point(x1, y1_pixel), cv::Point(x2, y2_pixel), cv::Scalar(0, 0, 255), 2);
  }

  // 绘制规划Yaw（蓝色）
  for (int i = 1; i < num_points; i++) {
    double t1 = time_points[i-1];
    double t2 = time_points[i];
    double y1 = debug.plan_yaw_list[i-1];
    double y2 = debug.plan_yaw_list[i];

    int x1 = margin + static_cast<int>((t1 + 0.01) / 0.51 * (width - 2 * margin));
    int x2 = margin + static_cast<int>((t2 + 0.01) / 0.51 * (width - 2 * margin));
    int y1_pixel = yaw_bottom - static_cast<int>(((y1 - min_yaw) / yaw_range) * (graph_height - 20));
    int y2_pixel = yaw_bottom - static_cast<int>(((y2 - min_yaw) / yaw_range) * (graph_height - 20));

    cv::line(image, cv::Point(x1, y1_pixel), cv::Point(x2, y2_pixel), cv::Scalar(255, 0, 0), 2);
  }

  // ========== Pitch ==========
  double min_pitch = *std::min_element(debug.target_pitch_list.begin(), debug.target_pitch_list.end());
  double max_pitch = *std::max_element(debug.target_pitch_list.begin(), debug.target_pitch_list.end());
  double pitch_range = max_pitch - min_pitch;
  if (pitch_range < 0.01) pitch_range = 0.01;
  min_pitch -= pitch_range * 0.1;
  max_pitch += pitch_range * 0.1;
  pitch_range = max_pitch - min_pitch;

  // 绘制目标Pitch（红色）
  for (int i = 1; i < num_points; i++) {
    double t1 = time_points[i-1];
    double t2 = time_points[i];
    double y1 = debug.target_pitch_list[i-1];
    double y2 = debug.target_pitch_list[i];

    int x1 = margin + static_cast<int>((t1 + 0.01) / 0.51 * (width - 2 * margin));
    int x2 = margin + static_cast<int>((t2 + 0.01) / 0.51 * (width - 2 * margin));
    int y1_pixel = pitch_bottom - static_cast<int>(((y1 - min_pitch) / pitch_range) * (graph_height - 20));
    int y2_pixel = pitch_bottom - static_cast<int>(((y2 - min_pitch) / pitch_range) * (graph_height - 20));

    cv::line(image, cv::Point(x1, y1_pixel), cv::Point(x2, y2_pixel), cv::Scalar(0, 0, 255), 2);
  }

  // 绘制规划Pitch（蓝色）
  for (int i = 1; i < num_points; i++) {
    double t1 = time_points[i-1];
    double t2 = time_points[i];
    double y1 = debug.plan_pitch_list[i-1];
    double y2 = debug.plan_pitch_list[i];

    int x1 = margin + static_cast<int>((t1 + 0.01) / 0.51 * (width - 2 * margin));
    int x2 = margin + static_cast<int>((t2 + 0.01) / 0.51 * (width - 2 * margin));
    int y1_pixel = pitch_bottom - static_cast<int>(((y1 - min_pitch) / pitch_range) * (graph_height - 20));
    int y2_pixel = pitch_bottom - static_cast<int>(((y2 - min_pitch) / pitch_range) * (graph_height - 20));

    cv::line(image, cv::Point(x1, y1_pixel), cv::Point(x2, y2_pixel), cv::Scalar(255, 0, 0), 2);
  }

  // ========== 发射点 ==========
  for (int i = 0; i < num_points; i++) {
    if (debug.fireable_list[i]) {
      double t = time_points[i];
      int x_pos = margin + static_cast<int>((t + 0.01) / 0.51 * (width - 2 * margin));

      double yaw = debug.plan_yaw_list[i];
      int y_yaw = yaw_bottom - static_cast<int>(((yaw - min_yaw) / yaw_range) * (graph_height - 20));
      cv::circle(image, cv::Point(x_pos, y_yaw), 5, cv::Scalar(0, 255, 0), -1);

      double pitch = debug.plan_pitch_list[i];
      int y_pitch = pitch_bottom - static_cast<int>(((pitch - min_pitch) / pitch_range) * (graph_height - 20));
      cv::circle(image, cv::Point(x_pos, y_pitch), 5, cv::Scalar(0, 255, 0), -1);
    }
  }

  // ========== 标签 ==========
  cv::putText(image, "Time (s)", cv::Point(width/2 - 30, height - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
  cv::putText(image, "Yaw (rad)", cv::Point(15, yaw_top + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
  cv::putText(image, "Pitch (rad)", cv::Point(15, pitch_top + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));

  int legend_x = width - 180;
  cv::rectangle(image, cv::Point(legend_x - 10, 10), cv::Point(width - 10, 110), cv::Scalar(240, 240, 240), -1);
  cv::rectangle(image, cv::Point(legend_x - 10, 10), cv::Point(width - 10, 110), cv::Scalar(200, 200, 200), 1);

  cv::line(image, cv::Point(legend_x, 30), cv::Point(legend_x + 25, 30), cv::Scalar(0, 0, 255), 3);
  cv::putText(image, "Target", cv::Point(legend_x + 35, 35), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));

  cv::line(image, cv::Point(legend_x, 55), cv::Point(legend_x + 25, 55), cv::Scalar(255, 0, 0), 3);
  cv::putText(image, "Planner", cv::Point(legend_x + 35, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));

  cv::circle(image, cv::Point(legend_x + 12, 82), 5, cv::Scalar(0, 255, 0), -1);
  cv::putText(image, "Fireable", cv::Point(legend_x + 35, 87), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));

  char param_text[128];
  sprintf(param_text, "d=%.1fm w=%.1f rad/s", d, w);
  cv::putText(image, param_text, cv::Point(10, height - 25), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(100, 100, 100));

  cv::imshow("Explicit Deceleration Planner", image);
  cv::waitKey(0);

  return 0;
}