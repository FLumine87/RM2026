#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <fstream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// 尝试不同的include路径
#ifdef ROS2_INCLUDE_PATH
#include <auto_aim_interfaces/msg/armor.hpp>
#include <auto_aim_interfaces/msg/armors.hpp>
#include <auto_aim_interfaces/msg/target.hpp>
#include <auto_aim_interfaces/msg/gimbal.hpp>
#else
#include "auto_aim_interfaces/msg/armor.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/gimbal.hpp"
#endif

using namespace std::chrono_literals;

class ROS2Publisher : public rclcpp::Node
{
public:
  ROS2Publisher()
    : Node("auto_aim_test_publisher")
    , tf_publisher_(this->create_publisher<tf2_msgs::msg::TFMessage>("tf", 10))
    , image_publisher_(this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10))
    , armor_msg_publisher_(this->create_publisher<auto_aim_interfaces::msg::Armors>("armor_msg", 10))
    , target_msg_publisher_(this->create_publisher<auto_aim_interfaces::msg::Target>("target_msg", 10))
    , gimbal_msg_publisher_(this->create_publisher<auto_aim_interfaces::msg::Gimbal>("gimbal_msg", 10))
    , marker_pub_(this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", 10))
  {
    position_marker_.ns = "position";
    position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
    position_marker_.color.a = 1.0;
    position_marker_.color.g = 1.0;

    linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
    linear_v_marker_.ns = "linear_v";
    linear_v_marker_.scale.x = 0.03;
    linear_v_marker_.scale.y = 0.05;
    linear_v_marker_.scale.z = 0.0;
    linear_v_marker_.color.a = 1.0;
    linear_v_marker_.color.r = 1.0;
    linear_v_marker_.color.g = 1.0;

    angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
    angular_v_marker_.ns = "angular_v";
    angular_v_marker_.scale.x = 0.03;
    angular_v_marker_.scale.y = 0.05;
    angular_v_marker_.scale.z = 0.0;
    angular_v_marker_.color.a = 1.0;
    angular_v_marker_.color.b = 1.0;
    angular_v_marker_.color.g = 1.0;

    armor_marker_.ns = "armors";
    armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
    armor_marker_.scale.x = 0.03;
    armor_marker_.scale.y = 0.135;
    armor_marker_.scale.z = 0.125;
    armor_marker_.color.a = 1.0;
    armor_marker_.color.r = 1.0;
  }

  void publish_tf(const Eigen::Quaterniond & q, const Eigen::Vector3d & t)
  {
    tf2_msgs::msg::TFMessage tf_msg;
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = this->now();
    transform.header.frame_id = "world";
    transform.child_frame_id = "camera";

    transform.transform.translation.x = t.x();
    transform.transform.translation.y = t.y();
    transform.transform.translation.z = t.z();

    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    tf_msg.transforms.push_back(transform);
    tf_publisher_->publish(tf_msg);
  }

  void publish_image(const cv::Mat & img, const std::string & topic)
  {
    auto msg = std::make_shared<sensor_msgs::msg::Image>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "camera";
    msg->height = img.rows;
    msg->width = img.cols;
    msg->encoding = "bgr8";
    msg->is_bigendian = false;
    msg->step = img.cols * img.elemSize();
    msg->data.assign(img.data, img.data + img.total() * img.elemSize());

    if (topic == "camera/image_raw")
      image_publisher_->publish(*msg);
  }

  void publish_armor_msg(const std::vector<auto_aim::Armor> & armors)
  {
    auto msg = std::make_shared<auto_aim_interfaces::msg::Armors>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "world";
    
    for (const auto & armor : armors) {
      auto_aim_interfaces::msg::Armor armor_msg;
      armor_msg.number = std::to_string(static_cast<int>(armor.name));
      armor_msg.type = (armor.type == auto_aim::ArmorType::big) ? "big" : "small";
      armor_msg.distance_to_image_center = static_cast<float>(cv::norm(armor.center - cv::Point2f(640.0f, 360.0f)));
      
      armor_msg.pose.position.x = armor.xyz_in_world.x();
      armor_msg.pose.position.y = armor.xyz_in_world.y();
      armor_msg.pose.position.z = armor.xyz_in_world.z();
      
      armor_msg.pose.orientation.w = 1.0;
      armor_msg.pose.orientation.x = 0.0;
      armor_msg.pose.orientation.y = 0.0;
      armor_msg.pose.orientation.z = 0.0;
      
      for (const auto & pt : armor.points) {
        geometry_msgs::msg::Point kpt;
        kpt.x = pt.x;
        kpt.y = pt.y;
        kpt.z = 0.0;
        armor_msg.kpts.push_back(kpt);
      }
      
      msg->armors.push_back(armor_msg);
    }
    
    armor_msg_publisher_->publish(*msg);
  }

  void publish_target_msg(const auto_aim::Target & target)
  {
    auto msg = std::make_shared<auto_aim_interfaces::msg::Target>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "world";
    msg->tracking = true;
    msg->id = std::to_string(static_cast<int>(target.name));
    msg->armors_num = static_cast<int32_t>(target.armor_xyza_list().size());
    msg->position.x = target.ekf_x()[0];
    msg->position.y = target.ekf_x()[2];
    msg->position.z = target.ekf_x()[4];
    msg->velocity.x = target.ekf_x()[1];
    msg->velocity.y = target.ekf_x()[3];
    msg->velocity.z = target.ekf_x()[5];
    msg->yaw = target.ekf_x()[6];
    msg->v_yaw = target.ekf_x()[7];
    msg->radius_1 = 0.0;
    msg->radius_2 = 0.0;
    msg->dz = target.ekf_x()[4];
    
    Eigen::Vector3d position(target.ekf_x()[0], target.ekf_x()[2], target.ekf_x()[4]);
    Eigen::Vector3d ypd = tools::xyz2ypd(position);
    msg->ypd_yaw = static_cast<float>(ypd[0]);
    msg->ypd_pitch = static_cast<float>(ypd[1]);
    msg->ypd_distance = static_cast<float>(ypd[2]);
    
    target_msg_publisher_->publish(*msg);
  }

  void publish_empty_target_msg()
  {
    auto msg = std::make_shared<auto_aim_interfaces::msg::Target>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "world";
    msg->tracking = false;
    target_msg_publisher_->publish(*msg);
  }

  void publish_gimbal_msg(const io::Command & command)
  {
    auto msg = std::make_shared<auto_aim_interfaces::msg::Gimbal>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "gimbal";
    msg->control = command.control;
    msg->fire = command.shoot;
    msg->yaw = command.yaw;
    msg->yaw_vel = 0.0; // Command中没有速度和加速度信息
    msg->yaw_acc = 0.0;
    msg->pitch = command.pitch;
    msg->pitch_vel = 0.0;
    msg->pitch_acc = 0.0;
    
    gimbal_msg_publisher_->publish(*msg);
  }

  void publish_marker(
    bool tracking, const Eigen::Vector3d & position, const Eigen::Vector3d & velocity, 
    const Eigen::Vector3d & angular_velocity, const std::vector<Eigen::Vector3d> & armor_positions,
    const std::vector<double> & armor_yaws)
  {
    auto now = this->now();
    visualization_msgs::msg::MarkerArray marker_array;

    if (tracking) {
      position_marker_.header.stamp = now;
      position_marker_.header.frame_id = "world";
      position_marker_.action = visualization_msgs::msg::Marker::ADD;
      position_marker_.pose.position.x = position.x();
      position_marker_.pose.position.y = position.y();
      position_marker_.pose.position.z = position.z();
      position_marker_.pose.orientation.w = 1.0;

      linear_v_marker_.header.stamp = now;
      linear_v_marker_.header.frame_id = "world";
      linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
      linear_v_marker_.points.clear();
      geometry_msgs::msg::Point start_point, end_point;
      start_point.x = position.x();
      start_point.y = position.y();
      start_point.z = position.z();
      end_point.x = position.x() + velocity.x();
      end_point.y = position.y() + velocity.y();
      end_point.z = position.z() + velocity.z();
      linear_v_marker_.points.push_back(start_point);
      linear_v_marker_.points.push_back(end_point);

      angular_v_marker_.header.stamp = now;
      angular_v_marker_.header.frame_id = "world";
      angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
      angular_v_marker_.points.clear();
      start_point.x = position.x();
      start_point.y = position.y();
      start_point.z = position.z();
      end_point.x = position.x();
      end_point.y = position.y();
      end_point.z = position.z() + angular_velocity.z() / CV_PI;
      angular_v_marker_.points.push_back(start_point);
      angular_v_marker_.points.push_back(end_point);

      armor_marker_.header.stamp = now;
      armor_marker_.header.frame_id = "world";
      armor_marker_.action = visualization_msgs::msg::Marker::ADD;
      armor_marker_.scale.y = 0.135;
      for (size_t i = 0; i < armor_positions.size(); i++) {
        armor_marker_.id = i;
        armor_marker_.pose.position.x = armor_positions[i].x();
        armor_marker_.pose.position.y = armor_positions[i].y();
        armor_marker_.pose.position.z = armor_positions[i].z();
        
        double yaw = armor_yaws[i];
        armor_marker_.pose.orientation.w = std::cos(yaw / 2.0);
        armor_marker_.pose.orientation.x = 0.0;
        armor_marker_.pose.orientation.y = 0.0;
        armor_marker_.pose.orientation.z = std::sin(yaw / 2.0);
        marker_array.markers.push_back(armor_marker_);
      }

      marker_array.markers.push_back(position_marker_);
      marker_array.markers.push_back(linear_v_marker_);
      marker_array.markers.push_back(angular_v_marker_);
    } else {
      position_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
      linear_v_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
      angular_v_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
      armor_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
      marker_array.markers.push_back(armor_marker_);
    }

    marker_pub_->publish(marker_array);
  }

  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armor_msg_publisher_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_msg_publisher_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Gimbal>::SharedPtr gimbal_msg_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker linear_v_marker_;
  visualization_msgs::msg::Marker angular_v_marker_;
  visualization_msgs::msg::Marker armor_marker_;
};

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明 }"
  "{config-path c  | configs/demo.yaml | yaml配置文件的路径}"
  "{start-index s  | 0                 | 视频起始帧下标    }"
  "{end-index e    | 0                 | 视频结束帧下标    }"
  "{@input-path    | assets/demo/demo  | avi和txt文件的路径}";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");

  // 初始化ROS2
  rclcpp::init(argc, argv);
  auto ros2_publisher = std::make_shared<ROS2Publisher>();

  // 启动ROS2 spin线程
  std::atomic<bool> quit = false;
  std::thread ros2_spin_thread([&]() {
    while (!quit) {
      rclcpp::spin_some(ros2_publisher);
      std::this_thread::sleep_for(1ms);
    }
  });

  auto video_path = fmt::format("{}.avi", input_path);
  auto text_path = fmt::format("{}.txt", input_path);
  cv::VideoCapture video(video_path);
  std::ifstream text(text_path);

  auto_aim::YOLO yolo(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);

  cv::Mat img, drawing;
  auto t0 = std::chrono::steady_clock::now();

  auto_aim::Target last_target;
  io::Command last_command;
  double last_t = -1;

  video.set(cv::CAP_PROP_POS_FRAMES, start_index);
  for (int i = 0; i < start_index; i++) {
    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
  }

  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;

    video.read(img);
    if (img.empty()) break;

    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
    auto timestamp = t0 + std::chrono::microseconds(int(t * 1e6));

    /// 自瞄核心逻辑

    solver.set_R_gimbal2world({w, x, y, z});

    // 发布tf变换
    Eigen::Quaterniond q(w, x, y, z);
    Eigen::Vector3d t_camera2world = solver.R_gimbal2world().transpose() * Eigen::Vector3d(0.145, 0, 0.07);
    ros2_publisher->publish_tf(q, t_camera2world);

    auto yolo_start = std::chrono::steady_clock::now();
    auto armors = yolo.detect(img, frame_count);

    auto tracker_start = std::chrono::steady_clock::now();
    auto targets = tracker.track(armors, timestamp);

    auto aimer_start = std::chrono::steady_clock::now();
    auto command = aimer.aim(targets, timestamp, 27, false);

    if (
      !targets.empty() && aimer.debug_aim_point.valid &&
      std::abs(command.yaw - last_command.yaw) * 57.3 < 2)
      command.shoot = true;

    if (command.control) last_command = command;
    /// 调试输出

    auto finish = std::chrono::steady_clock::now();
    tools::logger()->info(
      "[{}] yolo: {:.1f}ms, tracker: {:.1f}ms, aimer: {:.1f}ms", frame_count,
      tools::delta_time(tracker_start, yolo_start) * 1e3,
      tools::delta_time(aimer_start, tracker_start) * 1e3,
      tools::delta_time(finish, aimer_start) * 1e3);

    tools::draw_text(
      img,
      fmt::format(
        "command is {},{:.2f},{:.2f},shoot:{}", command.control, command.yaw * 57.3,
        command.pitch * 57.3, command.shoot),
      {10, 60}, {154, 50, 205});

    Eigen::Quaternion<double> gimbal_q = {w, x, y, z};
    tools::draw_text(
      img,
      fmt::format(
        "gimbal yaw{:.2f}", (tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0) * 57.3)[0]),
      {10, 90}, {255, 255, 255});

    nlohmann::json data;

    // 装甲板原始观测数据
    data["armor_num"] = armors.size();
    if (!armors.empty()) {
      const auto & armor = armors.front();
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
      data["armor_center_x"] = armor.center_norm.x;
      data["armor_center_y"] = armor.center_norm.y;
    }

    auto yaw = tools::eulers(gimbal_q, 2, 1, 0)[0];
    data["gimbal_yaw"] = yaw * 57.3;
    data["cmd_yaw"] = command.yaw * 57.3;
    data["shoot"] = command.shoot;

    if (!targets.empty()) {
      auto target = targets.front();

      if (last_t == -1) {
        last_target = target;
        last_t = t;
        continue;
      }

      std::vector<Eigen::Vector4d> armor_xyza_list;

      // 当前帧target更新后
      armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      // aimer瞄准位置
      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid) tools::draw_points(img, image_points, {0, 0, 255});

      // 观测器内部数据
      Eigen::VectorXd x = target.ekf_x();
      data["x"] = x[0];
      data["vx"] = x[1];
      data["y"] = x[2];
      data["vy"] = x[3];
      data["z"] = x[4];
      data["vz"] = x[5];
      data["a"] = x[6] * 57.3;
      data["w"] = x[7];
      data["r"] = x[8];
      data["l"] = x[9];
      data["h"] = x[10];
      data["last_id"] = target.last_id;

      // 卡方检验数据
      data["residual_yaw"] = target.ekf().data.at("residual_yaw");
      data["residual_pitch"] = target.ekf().data.at("residual_pitch");
      data["residual_distance"] = target.ekf().data.at("residual_distance");
      data["residual_angle"] = target.ekf().data.at("residual_angle");
      data["nis"] = target.ekf().data.at("nis");
      data["nees"] = target.ekf().data.at("nees");
      data["nis_fail"] = target.ekf().data.at("nis_fail");
      data["nees_fail"] = target.ekf().data.at("nees_fail");
      data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");

      // 发布ROS2消息
      std::vector<auto_aim::Armor> armor_vector(armors.begin(), armors.end());
      ros2_publisher->publish_armor_msg(armor_vector);
      ros2_publisher->publish_target_msg(target);
      ros2_publisher->publish_gimbal_msg(command);

      // 发布标记
      Eigen::Vector3d position(target.ekf_x()[0], target.ekf_x()[2], target.ekf_x()[4]);
      Eigen::Vector3d velocity(target.ekf_x()[1], target.ekf_x()[3], target.ekf_x()[5]);
      Eigen::Vector3d angular_velocity(0, 0, target.ekf_x()[7]);
      
      std::vector<Eigen::Vector3d> armor_positions;
      std::vector<double> armor_yaws;
      for (const auto & xyza : armor_xyza_list) {
        armor_positions.push_back(xyza.head(3));
        armor_yaws.push_back(xyza[3]);
      }
      
      ros2_publisher->publish_marker(true, position, velocity, angular_velocity, armor_positions, armor_yaws);
    } else {
      // 发布空标记
      ros2_publisher->publish_marker(false, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), {}, {});
      ros2_publisher->publish_armor_msg({});
      ros2_publisher->publish_empty_target_msg();
    }

    plotter.plot(data);

    // 发布图像
    ros2_publisher->publish_image(img, "camera/image_raw");

    // cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    // cv::imshow("reprojection", img);
    // int key = cv::waitKey(30);
    // if (key == 'q') break;
    
    // 添加时间延迟，使视频播放速度接近正常速度
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }

  // 退出
  quit = true;
  if (ros2_spin_thread.joinable()) ros2_spin_thread.join();
  rclcpp::shutdown();

  return 0;
}
