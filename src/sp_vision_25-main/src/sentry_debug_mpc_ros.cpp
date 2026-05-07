#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <yaml-cpp/yaml.h>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
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
#include <auto_aim_interfaces/msg/target_sentry.hpp>
#include <auto_aim_interfaces/msg/gimbal.hpp>
#include <auto_aim_interfaces/msg/gimbal_feedback.hpp>
#include <auto_aim_interfaces/msg/sentry_status.hpp>
#include <auto_aim_interfaces/msg/from_decision.hpp>
#include <auto_aim_interfaces/msg/gimbal_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#else
#include "auto_aim_interfaces/msg/armor.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/target_sentry.hpp"
#include "auto_aim_interfaces/msg/gimbal.hpp"
#include "auto_aim_interfaces/msg/gimbal_feedback.hpp"
#include "auto_aim_interfaces/msg/sentry_status.hpp"
#include "auto_aim_interfaces/msg/from_decision.hpp"
#include "auto_aim_interfaces/msg/gimbal_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#endif

using namespace std::chrono_literals;

class ROS2Publisher : public rclcpp::Node
{
public:
  ROS2Publisher(const std::string & config_path)
    : Node("sentry_debug_publisher")
    , tf_publisher_(this->create_publisher<tf2_msgs::msg::TFMessage>("tf", 10))
    , image_publisher_(this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10))
    // , armor_publisher_(this->create_publisher<sensor_msgs::msg::Image>("armor/image", 10))
    // , target_publisher_(this->create_publisher<sensor_msgs::msg::Image>("target/image", 10))
    , armor_msg_publisher_(this->create_publisher<auto_aim_interfaces::msg::Armors>("armor_msg", 10))
    , target_msg_publisher_(this->create_publisher<auto_aim_interfaces::msg::Target>("target_msg", 10))
    , gimbal_msg_publisher_(this->create_publisher<auto_aim_interfaces::msg::Gimbal>("gimbal_msg", 10))
    , gimbal_feedback_publisher_(this->create_publisher<auto_aim_interfaces::msg::GimbalFeedback>("gimbal_feedback", 10))
    , marker_pub_(this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", 10))
    , joint_state_pub_(this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS()))
    , sentry_status_pub_(this->create_publisher<auto_aim_interfaces::msg::SentryStatus>("/from_sentry", 10))
    , target_sentry_pub_(this->create_publisher<auto_aim_interfaces::msg::TargetSentry>(
        "/tracker/target", rclcpp::QoS(10).best_effort()))
    , cmd_vel_sub_(this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&ROS2Publisher::cmd_vel_callback, this, std::placeholders::_1)))
  , posture_sub_(this->create_subscription<auto_aim_interfaces::msg::FromDecision>(
        "/posture_number", 10, std::bind(&ROS2Publisher::posture_callback, this, std::placeholders::_1)))
  , gimbal_command_sub_(this->create_subscription<auto_aim_interfaces::msg::GimbalCommand>(
        "/gimbal_command", 10, std::bind(&ROS2Publisher::gimbal_command_callback, this, std::placeholders::_1)))
  , speed_multiplier_sub_(this->create_subscription<std_msgs::msg::Float64>(
        "/speed_multiplier", 10, std::bind(&ROS2Publisher::speed_multiplier_callback, this, std::placeholders::_1)))
  {

    auto yaml = YAML::LoadFile(config_path);
    
    auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
    R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
    
    auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
    R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
    
    auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();
    t_camera2gimbal_ = Eigen::Vector3d(t_camera2gimbal_data.data());

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

  //================================================================
  // 功能块1: TF与图像发布
  //================================================================
  void publish_tf(const Eigen::Quaterniond & q_gimbal)
  {
    tf2_msgs::msg::TFMessage tf_msg;
    auto now = this->now();

    Eigen::Matrix3d R_imubody2imuabs = q_gimbal.toRotationMatrix();
    Eigen::Matrix3d R_gimbal2world = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
    Eigen::Quaterniond q_gimbal2world(R_gimbal2world);

    geometry_msgs::msg::TransformStamped transform_odom_gimbal;
    transform_odom_gimbal.header.stamp = now;
    transform_odom_gimbal.header.frame_id = "odom";
    transform_odom_gimbal.child_frame_id = "gimbal_link";
    transform_odom_gimbal.transform.translation.x = 0;
    transform_odom_gimbal.transform.translation.y = 0;
    transform_odom_gimbal.transform.translation.z = 0;
    transform_odom_gimbal.transform.rotation.x = q_gimbal2world.x();
    transform_odom_gimbal.transform.rotation.y = q_gimbal2world.y();
    transform_odom_gimbal.transform.rotation.z = q_gimbal2world.z();
    transform_odom_gimbal.transform.rotation.w = q_gimbal2world.w();
    tf_msg.transforms.push_back(transform_odom_gimbal);

    geometry_msgs::msg::TransformStamped transform_gimbal_camera;
    transform_gimbal_camera.header.stamp = now;
    transform_gimbal_camera.header.frame_id = "gimbal_link";
    transform_gimbal_camera.child_frame_id = "camera_link";
    transform_gimbal_camera.transform.translation.x = t_camera2gimbal_.x();
    transform_gimbal_camera.transform.translation.y = t_camera2gimbal_.y();
    transform_gimbal_camera.transform.translation.z = t_camera2gimbal_.z();
    
    Eigen::Quaterniond q_camera2gimbal(R_camera2gimbal_);
    transform_gimbal_camera.transform.rotation.x = q_camera2gimbal.x();
    transform_gimbal_camera.transform.rotation.y = q_camera2gimbal.y();
    transform_gimbal_camera.transform.rotation.z = q_camera2gimbal.z();
    transform_gimbal_camera.transform.rotation.w = q_camera2gimbal.w();
    tf_msg.transforms.push_back(transform_gimbal_camera);

    geometry_msgs::msg::TransformStamped transform_camera_optical;
    transform_camera_optical.header.stamp = now;
    transform_camera_optical.header.frame_id = "camera_link";
    transform_camera_optical.child_frame_id = "camera_optical_frame";
    transform_camera_optical.transform.translation.x = 0;
    transform_camera_optical.transform.translation.y = 0;
    transform_camera_optical.transform.translation.z = 0;
    
    Eigen::Quaterniond q_optical;
    q_optical = Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ());
    transform_camera_optical.transform.rotation.x = q_optical.x();
    transform_camera_optical.transform.rotation.y = q_optical.y();
    transform_camera_optical.transform.rotation.z = q_optical.z();
    transform_camera_optical.transform.rotation.w = q_optical.w();
    tf_msg.transforms.push_back(transform_camera_optical);

    tf_publisher_->publish(tf_msg);
  }

  void publish_image(const cv::Mat & img, const std::string & topic)
  {
    auto msg = std::make_shared<sensor_msgs::msg::Image>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "camera_optical_frame";
    msg->height = img.rows;
    msg->width = img.cols;
    msg->encoding = "bgr8";
    msg->is_bigendian = false;
    msg->step = img.cols * img.elemSize();
    msg->data.assign(img.data, img.data + img.total() * img.elemSize());

    if (topic == "camera/image_raw")
      image_publisher_->publish(*msg);
    // else if (topic == "armor/image")
    //   armor_publisher_->publish(*msg);
    // else if (topic == "target/image")
    //   target_publisher_->publish(*msg);
  }

  //================================================================
  // 功能块2: 视觉目标发布 (armor/Target/TargetSentry)
  //================================================================
  void publish_armor_msg(const std::vector<auto_aim::Armor> & armors)
  {
    auto msg = std::make_shared<auto_aim_interfaces::msg::Armors>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "odom";
    
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
    msg->header.frame_id = "odom";
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
    msg->header.frame_id = "odom";
    msg->tracking = false;
    target_msg_publisher_->publish(*msg);
  }

  void publish_target_sentry(const auto_aim::Target & target)
  {
    auto target_sentry_msg = std::make_shared<auto_aim_interfaces::msg::TargetSentry>();
    target_sentry_msg->header.stamp = this->now();
    target_sentry_msg->header.frame_id = "odom";
    target_sentry_msg->tracking = true;
    target_sentry_msg->position.x = target.ekf_x()[0];
    target_sentry_msg->position.y = target.ekf_x()[2];
    target_sentry_msg->position.z = target.ekf_x()[4];

    target_sentry_pub_->publish(*target_sentry_msg);
  }

  void publish_empty_target_sentry()
  {
    auto target_sentry_msg = std::make_shared<auto_aim_interfaces::msg::TargetSentry>();
    target_sentry_msg->header.stamp = this->now();
    target_sentry_msg->header.frame_id = "odom";
    target_sentry_msg->tracking = false;

    target_sentry_pub_->publish(*target_sentry_msg);
  }

  //================================================================
  // 功能块3: 云台状态发布 (feedback/joint_state/sentry_status)
  //================================================================
  void publish_gimbal_msg(const auto_aim::Plan & plan)
  {
    auto msg = std::make_shared<auto_aim_interfaces::msg::Gimbal>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "gimbal";
    msg->control = plan.control;
    msg->fire = plan.fire;
    msg->yaw = plan.yaw;
    msg->yaw_vel = plan.yaw_vel;
    msg->yaw_acc = plan.yaw_acc;
    msg->pitch = plan.pitch;
    msg->pitch_vel = plan.pitch_vel;
    msg->pitch_acc = plan.pitch_acc;
    
    gimbal_msg_publisher_->publish(*msg);
  }

  void publish_gimbal_feedback(const io::GimbalState & state, uint8_t mode, const Eigen::Quaterniond & q)
  {
    auto msg = std::make_shared<auto_aim_interfaces::msg::GimbalFeedback>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "gimbal";
    msg->mode = mode;
    msg->q[0] = static_cast<float>(q.w());
    msg->q[1] = static_cast<float>(q.x());
    msg->q[2] = static_cast<float>(q.y());
    msg->q[3] = static_cast<float>(q.z());
    msg->yaw = state.yaw;
    msg->yaw_vel = state.yaw_vel;
    msg->pitch = state.pitch;
    msg->pitch_vel = state.pitch_vel;
    msg->bullet_speed = state.bullet_speed;
    msg->bullet_count = state.bullet_count;
    
    gimbal_feedback_publisher_->publish(*msg);
  }

  void publish_joint_state(const io::GimbalState & state)
  {
    auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
    joint_state_msg->header.stamp = this->now();
    joint_state_msg->name = {
      "gimbal_pitch_joint",
      "gimbal_yaw_joint",
      "gimbal_pitch_odom_joint",
      "gimbal_yaw_odom_joint",
      "front_left_wheel_joint",
      "front_right_wheel_joint",
      "rear_left_wheel_joint",
      "rear_right_wheel_joint"
    };
    joint_state_msg->position = {
      state.pitch,
      state.yaw,
      0,
      0,
      0,
      0,
      0,
      0
    };
    
    joint_state_pub_->publish(*joint_state_msg);
  }

  void publish_sentry_status(const io::GimbalState & state)
  {
    auto sentry_status_msg = std::make_shared<auto_aim_interfaces::msg::SentryStatus>();
    sentry_status_msg->is_play = state.is_play;
    sentry_status_msg->time = state.time_;
    sentry_status_msg->own_hp = state.own_hp_;
    sentry_status_msg->allowance = state.bullet_count;
    sentry_status_msg->outpost_hp = state.outpost_HP;
    sentry_status_msg->mode = state.mode;
    sentry_status_msg->reverse = state.reverse;
    
    sentry_status_pub_->publish(*sentry_status_msg);
  }

  //================================================================
  // 功能块4: 导航数据订阅回调与获取
  //================================================================
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    nav_vx_ = msg->linear.x;
    nav_vy_ = msg->linear.y;
  }

  void posture_callback(const auto_aim_interfaces::msg::FromDecision::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    nav_posture_ = msg->posture;
    nav_spin_flag_ = msg->spin_flag;
    nav_scan_ = msg->scan;
  }

  void gimbal_command_callback(const auto_aim_interfaces::msg::GimbalCommand::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    nav_gimbal_yaw_ = msg->yaw;
    nav_gimbal_pitch_ = msg->pitch;
    nav_reverse_ = msg->reverse;
    has_gimbal_command_ = true;
    last_gimbal_command_time_ = this->now();
  }

  void speed_multiplier_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    double new_multiplier = msg->data;
    if (new_multiplier < 0.0) new_multiplier = 0.0;
    if (new_multiplier > 1.0) new_multiplier = 1.0;
    speed_multiplier_ = new_multiplier;
  }

  float get_nav_vx() const
  {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    return nav_vx_;
  }

  float get_nav_vy() const
  {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    return nav_vy_;
  }

  uint8_t get_nav_posture() const
  {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    return nav_posture_;
  }

  uint8_t get_nav_spin_flag() const
  {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    return nav_spin_flag_;
  }

  uint8_t get_nav_scan() const
  {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    return nav_scan_;
  }

  float get_nav_gimbal_yaw() const
  {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    return nav_gimbal_yaw_;
  }

  float get_nav_gimbal_pitch() const
  {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    return nav_gimbal_pitch_;
  }

  uint8_t get_nav_reverse() const
  {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    return nav_reverse_;
  }

  bool get_has_gimbal_command() const
  {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    return has_gimbal_command_;
  }

  rclcpp::Time get_last_gimbal_command_time() const
  {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    return last_gimbal_command_time_;
  }

  double get_speed_multiplier() const
  {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    return speed_multiplier_;
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
      position_marker_.header.frame_id = "odom";
      position_marker_.action = visualization_msgs::msg::Marker::ADD;
      position_marker_.pose.position.x = position.x();
      position_marker_.pose.position.y = position.y();
      position_marker_.pose.position.z = position.z();
      position_marker_.pose.orientation.w = 1.0;

      linear_v_marker_.header.stamp = now;
      linear_v_marker_.header.frame_id = "odom";
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
      angular_v_marker_.header.frame_id = "odom";
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
      armor_marker_.header.frame_id = "odom";
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

  //================================================================
  // 功能块5: 可视化标记发布 (RViz标记)
  //================================================================

  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr armor_publisher_;
  // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr target_publisher_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armor_msg_publisher_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_msg_publisher_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Gimbal>::SharedPtr gimbal_msg_publisher_;
  rclcpp::Publisher<auto_aim_interfaces::msg::GimbalFeedback>::SharedPtr gimbal_feedback_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::SentryStatus>::SharedPtr sentry_status_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::TargetSentry>::SharedPtr target_sentry_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::FromDecision>::SharedPtr posture_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::GimbalCommand>::SharedPtr gimbal_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_multiplier_sub_;
  
  // 导航相关变量
  mutable std::mutex nav_mutex_;
  float nav_vx_ = 0.0f;
  float nav_vy_ = 0.0f;
  uint8_t nav_posture_ = 0;
  uint8_t nav_spin_flag_ = 0;
  uint8_t nav_scan_ = 0;
  float nav_gimbal_yaw_ = 0.0f;
  float nav_gimbal_pitch_ = 0.0f;
  uint8_t nav_reverse_ = 0;
  bool has_gimbal_command_ = false;
  rclcpp::Time last_gimbal_command_time_;
  double speed_multiplier_ = 0.8;

  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker linear_v_marker_;
  visualization_msgs::msg::Marker angular_v_marker_;
  visualization_msgs::msg::Marker armor_marker_;

  Eigen::Matrix3d R_gimbal2imubody_;
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;
};

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  rclcpp::init(argc, argv);
  auto ros2_publisher = std::make_shared<ROS2Publisher>(config_path);

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  std::mutex plan_mutex;
  auto_aim::Plan latest_plan = {};

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  std::atomic<bool> quit = false;
  std::thread ros2_spin_thread([&]() { rclcpp::spin(ros2_publisher); });

  //================================================================
  // 功能块: 规划线程 - 视觉规划 + 导航数据获取 + 下行发送
  //================================================================
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      auto target = target_queue.front();
      auto gs = gimbal.state();
      auto plan = planner.plan(target, gs.bullet_speed);

      {
        std::lock_guard<std::mutex> lock(plan_mutex);
        latest_plan = plan;
      }

      // 获取导航数据
      double current_multiplier = ros2_publisher->get_speed_multiplier();
      float vx = ros2_publisher->get_nav_vx() * current_multiplier;
      float vy = -ros2_publisher->get_nav_vy() * current_multiplier;
      uint8_t posture = ros2_publisher->get_nav_posture();
      uint8_t spin_flag = ros2_publisher->get_nav_spin_flag();
      uint8_t scan = ros2_publisher->get_nav_scan();
      uint8_t reverse = ros2_publisher->get_nav_reverse();

      // 检查是否收到云台指令，以及指令是否在超时时间内（500ms）
      bool use_gimbal_command = false;
      float yaw_command = plan.yaw;
      float pitch_command = plan.pitch;
      if (ros2_publisher->get_has_gimbal_command()) {
        auto now = ros2_publisher->get_clock()->now();
        auto elapsed = now - ros2_publisher->get_last_gimbal_command_time();
        if (elapsed.seconds() < 0.5) {
          use_gimbal_command = true;
          yaw_command = ros2_publisher->get_nav_gimbal_yaw();
          pitch_command = ros2_publisher->get_nav_gimbal_pitch();
        }
      }

      io::VisionToGimbal msg;
      msg.mode = plan.control ? (plan.fire ? 2 : 1) : 0;
      msg.yaw = yaw_command;
      msg.yaw_vel = plan.yaw_vel;
      msg.yaw_acc = plan.yaw_acc;
      msg.pitch = pitch_command;
      msg.pitch_vel = plan.pitch_vel;
      msg.pitch_acc = plan.pitch_acc;
      msg.vx = vx;
      msg.vy = vy;
      msg.posture = posture;
      msg.spin_flag = spin_flag;
      msg.scan = scan;
      msg.reverse = reverse;

      gimbal.send(msg);

      auto fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;

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
        data["target_z"] = target->ekf_x()[4];
        data["target_vz"] = target->ekf_x()[5];
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

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

  //================================================================
  // 功能块: 主循环 - 图像获取、检测跟踪、ROS上行发布、可视化
  //================================================================
  while (!exiter.exit()) {
    camera.read(img, t);
    auto q = gimbal.q(t);
    auto gs = gimbal.state();
    auto mode = gimbal.mode();

    solver.set_R_gimbal2world(q);
    
    ros2_publisher->publish_tf(q);
    
//================================================================
    // 发布云台反馈消息
    uint8_t mode_value = 0;
    switch (mode) {
      case io::GimbalMode::IDLE: mode_value = 0; break;
      case io::GimbalMode::AUTO_AIM: mode_value = 1; break;
      case io::GimbalMode::SMALL_BUFF: mode_value = 2; break;
      case io::GimbalMode::BIG_BUFF: mode_value = 3; break;
    }
    ros2_publisher->publish_gimbal_feedback(gs, mode_value, q);
    
    // 发布关节状态和哨兵状态消息
    ros2_publisher->publish_joint_state(gs);
    ros2_publisher->publish_sentry_status(gs);
//================================================================  
  
    auto armors = yolo.detect(img);
    auto targets = tracker.track(armors, t);
    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);

    if (!targets.empty()) {
      auto target = targets.front();

      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      Eigen::Vector4d aim_xyza = planner.debug_xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      tools::draw_points(img, image_points, {0, 0, 255});

      // 发布装甲板消息
      std::vector<auto_aim::Armor> armor_vector(armors.begin(), armors.end());
      ros2_publisher->publish_armor_msg(armor_vector);

      // 发布目标消息
      ros2_publisher->publish_target_msg(target);

      // 发布TargetSentry消息
      ros2_publisher->publish_target_sentry(target);

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
      ros2_publisher->publish_marker(false, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), {}, {});
      ros2_publisher->publish_armor_msg({});
      ros2_publisher->publish_empty_target_msg();
      // 发布空TargetSentry消息
      ros2_publisher->publish_empty_target_sentry();
    }

    {
      std::lock_guard<std::mutex> lock(plan_mutex);
      auto text = fmt::format(
        "yaw: {:.2f} pitch: {:.2f} fire: {}", latest_plan.yaw, latest_plan.pitch, latest_plan.fire);
      cv::putText(img, text, {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);
      // 发布云台控制消息
      ros2_publisher->publish_gimbal_msg(latest_plan);
    }

    auto delay_text = fmt::format(
      "delay: {:.1f} ms", tools::delta_time(std::chrono::steady_clock::now(), t) * 1000.0);
    cv::putText(img, delay_text, {10, 60}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);

    ros2_publisher->publish_image(img, "camera/image_raw");

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  if (ros2_spin_thread.joinable()) ros2_spin_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
  rclcpp::shutdown();

  return 0;
}
// ls -la /dev/shm/fastrtps*
// sudo rm -f /dev/shm/fastrtps*
