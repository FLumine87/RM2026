#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <map>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/bullet_detector/detect_bullet.hpp"
#include "tasks/bullet_detector/do_reproj.hpp"
#include "tools/coord_converter.hpp"
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
#include <auto_aim_interfaces/msg/gimbal_feedback.hpp>
#else
#include "auto_aim_interfaces/msg/armor.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/gimbal.hpp"
#include "auto_aim_interfaces/msg/gimbal_feedback.hpp"
#endif

using namespace std::chrono_literals;

// 弹道轨迹预测参数
const double GRAVITY = 9.78; // 重力加速度 (m/s^2)
const double AIR_RESISTANCE_FACTOR = 0.022; // 空气阻力因子
const double BULLET_V0 = 20.0; // 弹丸初速度 (m/s)

// 延迟补偿参数
const double SYSTEM_LATENCY = 0.01; // 系统延迟 (秒)

// 子弹数量限制
const int MAX_BULLET_COUNT = 5; // 最大子弹数量

// 发射原点与相机系原点的偏差
const Eigen::Vector3d LAUNCH_ORIGIN_OFFSET(0.05, 0.00, 0.0); // 空间偏差 (米)xyz右下前
const Eigen::Vector3d LAUNCH_DIRECTION_OFFSET(0.0, 0.0, 0.0); // 朝向偏差 (弧度)ypr

// 弹丸历史信息结构体
struct BulletHistory {
    cv::Point2f last_position; // 上一帧的弹丸位置
    double last_time; // 上一帧的时间
    int consecutive_detections; // 连续检测到的次数
    double estimated_distance; // 估算的弹丸距离
    double estimated_speed; // 估算的弹丸速度
    double trajectory_cost; // 与理想弹道的偏差成本
    
    BulletHistory() : last_time(0), consecutive_detections(0), estimated_distance(10.0), estimated_speed(0.0), trajectory_cost(0.0) {}
    BulletHistory(const cv::Point2f& pos, double t, double distance) : 
        last_position(pos), last_time(t), consecutive_detections(1), estimated_distance(distance), estimated_speed(0.0), trajectory_cost(0.0) {}
};

// 弹丸速度范围
const double MIN_BULLET_SPEED = 5.0; // 最小速度 5m/s
const double MAX_BULLET_SPEED = 30.0; // 最大速度 30m/s

// 实际弹丸半径（米）
const double ACTUAL_BULLET_RADIUS = 0.0085; // 8.5mm

// 弹丸过滤参数
const double MIN_BULLET_DISTANCE = 0.1; // 最小距离 0.1m
const double MAX_BULLET_DISTANCE = 20.0; // 最大距离 20m
const double BULLET_BRIGHTNESS_THRESHOLD = 30.0; // 弹丸亮度阈值
const double BULLET_COLOR_SATURATION_THRESHOLD = 20.0; // 弹丸颜色饱和度阈值
const double BULLET_MATCH_DISTANCE_THRESHOLD = 80.0; // 弹丸匹配距离阈值（像素）
const double BULLET_MAX_TIME_DIFF = 0.8; // 弹丸最大时间差（秒）

// 字体大小参数
const double FONT_SIZE_SCALE = 0.2; // 字体大小缩放因子（减小）
const double MIN_FONT_SIZE = 0.3; // 最小字体大小（减小，改为double类型）
const double MAX_FONT_SIZE = 0.8; // 最大字体大小（减小，改为double类型）

// 弹丸ID生成器
int next_bullet_id = 1;

// 弹丸ID到历史信息的映射
std::map<int, BulletHistory> bullet_histories;

// 弹道轨迹预测类
class ProjectileTrajectoryPredictor {
public:
    ProjectileTrajectoryPredictor(aimer::CoordConverter* converter) : converter_(converter) {}
    
    // 计算理想弹道上的点（相机坐标系）
    Eigen::Vector3d calculateIdealTrajectoryPoint(double time, double initial_speed) {
        double t = time;
        
        // 发射朝向偏移（ypr格式）
        double yaw = LAUNCH_DIRECTION_OFFSET.x();
        double pitch = LAUNCH_DIRECTION_OFFSET.y();
        
        // 计算理想位置（相机坐标系：z向前，x向右，y向下）
        Eigen::Vector3d position;
        
        // 修正速度分量计算
        double vx = initial_speed * sin(yaw) * cos(pitch);      // 水平向右速度
        double vy = initial_speed * sin(pitch);                 // 垂直向下速度（相机坐标系y轴向下为正）
        double vz = initial_speed * cos(pitch) * cos(yaw);      // 向前速度
        
        // 位置计算（考虑重力）
        position.x() = vx * t;
        position.y() = vy * t + 0.5 * GRAVITY * t * t;  // 重力向下的位移
        position.z() = vz * t;
        
        // 添加发射原点偏移（相机坐标系）
        position += LAUNCH_ORIGIN_OFFSET;
        
        return position;
    }
    
    // 计算弹丸与理想弹道的偏差成本
    double calculateTrajectoryCost(const cv::Point2f& bullet_pos, double time, double estimated_speed) {
        // 计算理想弹道上的点
        Eigen::Vector3d ideal_position = calculateIdealTrajectoryPoint(time, estimated_speed);
        
        // 将理想位置投影到图像平面
        cv::Point2f ideal_image_pos = converter_->pc_to_pu(ideal_position);
        
        // 计算距离成本
        double distance_cost = cv::norm(bullet_pos - ideal_image_pos);
        
        return distance_cost;
    }
    
    // 绘制理想弹道的抛物线
    void drawIdealTrajectory(cv::Mat& img, double initial_speed, double max_time = -1.0, int num_points = 50) {
        // 如果未指定max_time，根据弹丸能飞行的最远近距离自动计算
        if (max_time <= 0) {
            double pitch = LAUNCH_DIRECTION_OFFSET.y();
            double vy_initial = initial_speed * sin(pitch);
            
            if (vy_initial > 0) {
                max_time = (2 * vy_initial) / GRAVITY;
            } else {
                max_time = sqrt((2 * std::abs(LAUNCH_ORIGIN_OFFSET.y())) / GRAVITY);
            }
            
            max_time = std::max(0.5, std::min(max_time, 5.0));
        }
        
        std::vector<cv::Point2f> trajectory_points;
        
        // 计算弹道上的多个点
        double time_step = max_time / num_points;
        for (int i = 0; i <= num_points; ++i) {
            double time = i * time_step;
            Eigen::Vector3d position_cam = calculateIdealTrajectoryPoint(time, initial_speed);
            
            // 使用正确的相机投影方法
            cv::Point2f image_pos = converter_->pc_to_pu(position_cam);
            
            // 只添加在图像范围内的点
            if (image_pos.x >= 0 && image_pos.x < img.cols && image_pos.y >= 0 && image_pos.y < img.rows) {
                trajectory_points.push_back(image_pos);
            }
        }
        
        // 绘制抛物线
        if (trajectory_points.size() > 1) {
            // 绘制轨迹线段，距离越远线越细
            std::vector<Eigen::Vector3d> cam_positions;
            double time_step = max_time / num_points;
            for (int i = 0; i <= num_points; ++i) {
                double time = i * time_step;
                Eigen::Vector3d position_cam = calculateIdealTrajectoryPoint(time, initial_speed);
                cam_positions.push_back(position_cam);
            }
            
            // 绘制动态粗细的线段
            for (size_t i = 0; i < trajectory_points.size() - 1; ++i) {
                double distance = cam_positions[i].z();
                double line_thickness = std::max(0.5, 4.0 / (1.0 + distance * 0.2));
                
                cv::line(img, trajectory_points[i], trajectory_points[i+1], cv::Scalar(255, 0, 255), line_thickness, cv::LINE_AA);
            }
            
            // 绘制轨迹起点（绿色圆点）
            cv::circle(img, trajectory_points.front(), 8, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
        }
    }
    
private:
    aimer::CoordConverter* converter_;
};

class ROS2Publisher : public rclcpp::Node
{
public:
  ROS2Publisher(ProjectileTrajectoryPredictor* trajectory_predictor = nullptr)
    : Node("auto_aim_bullet_detector_publisher")
    , trajectory_predictor_(trajectory_predictor)
    , tf_publisher_(this->create_publisher<tf2_msgs::msg::TFMessage>("tf", 10))
    , image_publisher_(this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10))
    , armor_msg_publisher_(this->create_publisher<auto_aim_interfaces::msg::Armors>("armor_msg", 10))
    , target_msg_publisher_(this->create_publisher<auto_aim_interfaces::msg::Target>("target_msg", 10))
    , gimbal_msg_publisher_(this->create_publisher<auto_aim_interfaces::msg::Gimbal>("gimbal_msg", 10))
    , gimbal_feedback_publisher_(this->create_publisher<auto_aim_interfaces::msg::GimbalFeedback>("gimbal_feedback", 10))
    , marker_pub_(this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", 10))
  {
    position_marker_.ns = "position";
    position_marker_.id = 1;
    position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
    position_marker_.color.a = 1.0;
    position_marker_.color.g = 1.0;

    linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
    linear_v_marker_.ns = "linear_v";
    linear_v_marker_.id = 2;
    linear_v_marker_.scale.x = 0.03;
    linear_v_marker_.scale.y = 0.05;
    linear_v_marker_.scale.z = 0.0;
    linear_v_marker_.color.a = 1.0;
    linear_v_marker_.color.r = 1.0;
    linear_v_marker_.color.g = 1.0;

    angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
    angular_v_marker_.ns = "angular_v";
    angular_v_marker_.id = 3;
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

    // 初始化轨迹marker
    init_trajectory_marker();
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

  void publish_marker(
    bool tracking, const Eigen::Vector3d & position, const Eigen::Vector3d & velocity,
    const Eigen::Vector3d & angular_velocity, const std::vector<Eigen::Vector3d> & armor_positions,
    const std::vector<double> & armor_yaws, const Eigen::Quaterniond & q_gimbal = Eigen::Quaterniond::Identity())
  {
    auto now = this->now();
    visualization_msgs::msg::MarkerArray marker_array;

    if (tracking) {
      // 完整初始化位置marker
      position_marker_.header.stamp = now;
      position_marker_.header.frame_id = "world";
      position_marker_.ns = "position";
      position_marker_.id = 1;
      position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
      position_marker_.action = visualization_msgs::msg::Marker::ADD;
      position_marker_.scale.x = 0.1;
      position_marker_.scale.y = 0.1;
      position_marker_.scale.z = 0.1;
      position_marker_.color.a = 1.0;
      position_marker_.color.g = 1.0;
      position_marker_.pose.position.x = position.x();
      position_marker_.pose.position.y = position.y();
      position_marker_.pose.position.z = position.z();
      position_marker_.pose.orientation.w = 1.0;

      // 完整初始化线速度marker
      linear_v_marker_.header.stamp = now;
      linear_v_marker_.header.frame_id = "world";
      linear_v_marker_.ns = "linear_v";
      linear_v_marker_.id = 2;
      linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
      linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
      linear_v_marker_.scale.x = 0.03;
      linear_v_marker_.scale.y = 0.05;
      linear_v_marker_.scale.z = 0.0;
      linear_v_marker_.color.a = 1.0;
      linear_v_marker_.color.r = 1.0;
      linear_v_marker_.color.g = 1.0;
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

      // 完整初始化角速度marker
      angular_v_marker_.header.stamp = now;
      angular_v_marker_.header.frame_id = "world";
      angular_v_marker_.ns = "angular_v";
      angular_v_marker_.id = 3;
      angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
      angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
      angular_v_marker_.scale.x = 0.03;
      angular_v_marker_.scale.y = 0.05;
      angular_v_marker_.scale.z = 0.0;
      angular_v_marker_.color.a = 1.0;
      angular_v_marker_.color.b = 1.0;
      angular_v_marker_.color.g = 1.0;
      angular_v_marker_.points.clear();
      start_point.x = position.x();
      start_point.y = position.y();
      start_point.z = position.z();
      end_point.x = position.x();
      end_point.y = position.y();
      end_point.z = position.z() + angular_velocity.z() / CV_PI;
      angular_v_marker_.points.push_back(start_point);
      angular_v_marker_.points.push_back(end_point);

      // 先发布轨迹marker
      publish_trajectory_marker(now, marker_array, q_gimbal);


      // 然后添加其他marker
      marker_array.markers.push_back(position_marker_);
      marker_array.markers.push_back(linear_v_marker_);
      marker_array.markers.push_back(angular_v_marker_);

      // 最后添加装甲板marker
      armor_marker_.header.stamp = now;
      armor_marker_.header.frame_id = "world";
      armor_marker_.action = visualization_msgs::msg::Marker::ADD;
      armor_marker_.scale.y = 0.135;
      for (size_t i = 0; i < armor_positions.size(); i++) {
        armor_marker_.id = static_cast<int>(i) + 4; // 确保id唯一
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
    } else {
      // 删除装甲板相关的marker，但保留弹道轨迹marker
      position_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
      linear_v_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
      angular_v_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
      armor_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
      marker_array.markers.push_back(armor_marker_);
      
      // 即使没有检测到装甲板，也发布弹道轨迹marker
      publish_trajectory_marker(now, marker_array, q_gimbal);
    }

    marker_pub_->publish(marker_array);
  }

  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armor_msg_publisher_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_msg_publisher_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Gimbal>::SharedPtr gimbal_msg_publisher_;
  rclcpp::Publisher<auto_aim_interfaces::msg::GimbalFeedback>::SharedPtr gimbal_feedback_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // 弹道轨迹marker
  visualization_msgs::msg::Marker trajectory_marker_;
  ProjectileTrajectoryPredictor* trajectory_predictor_;

  void set_trajectory_predictor(ProjectileTrajectoryPredictor* predictor) {
    trajectory_predictor_ = predictor;
  }

  void init_trajectory_marker()
  {
    trajectory_marker_.ns = "trajectory";
    trajectory_marker_.id = 0;
    trajectory_marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trajectory_marker_.scale.x = 0.02;
    trajectory_marker_.color.a = 1.0;
    trajectory_marker_.color.r = 1.0;
    trajectory_marker_.color.g = 0.0;
    trajectory_marker_.color.b = 1.0;
  }

  void publish_trajectory_marker(const rclcpp::Time & now, visualization_msgs::msg::MarkerArray & marker_array, const Eigen::Quaterniond & q_gimbal)
  {
    if (!trajectory_predictor_) return;
    
    trajectory_marker_.header.stamp = now;
    trajectory_marker_.header.frame_id = "world";
    trajectory_marker_.ns = "trajectory";
    trajectory_marker_.id = 0;
    trajectory_marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trajectory_marker_.action = visualization_msgs::msg::Marker::ADD;
    trajectory_marker_.scale.x = 0.02;
    trajectory_marker_.color.a = 1.0;
    trajectory_marker_.color.r = 1.0;
    trajectory_marker_.color.g = 0.0;
    trajectory_marker_.color.b = 1.0;
    trajectory_marker_.points.clear();

    // 相机坐标系到世界坐标系的转换
    // 相机坐标系：z向前，x向右，y向下
    // 世界坐标系：z向上，x向前，y向左（ROS标准）
    Eigen::Matrix3d R_cam_to_world = q_gimbal.toRotationMatrix();
    
    // 最大距离
    const double max_distance = MAX_BULLET_DISTANCE; // 20米
    
    // 计算弹道轨迹点（抛物线）
    int num_points = 200; // 轨迹点数量
    double total_time = max_distance / BULLET_V0; // 总飞行时间
    double time_step = total_time / num_points;
    
    for (int i = 0; i <= num_points; ++i) {
      double time = i * time_step;
      Eigen::Vector3d position_cam = trajectory_predictor_->calculateIdealTrajectoryPoint(time, BULLET_V0);
      
      // 转换到世界坐标系：需要将相机坐标系转换为ROS世界坐标系
      // 相机坐标系：z向前，x向右，y向下
      // 世界坐标系：z向上，x向前，y向左
      // 转换关系：world_x = cam_z, world_y = -cam_x, world_z = -cam_y
      Eigen::Vector3d position_world;
      position_world.x() = position_cam.z();  // 相机z向前 -> 世界x向前
      position_world.y() = -position_cam.x(); // 相机x向右 -> 世界y向左（取负）
      position_world.z() = -position_cam.y(); // 相机y向下 -> 世界z向上（取负）
      
      // 应用云台姿态旋转
      position_world = R_cam_to_world * position_world;
      
      geometry_msgs::msg::Point point;
      point.x = position_world.x();
      point.y = position_world.y();
      point.z = position_world.z();
      trajectory_marker_.points.push_back(point);
      
      // 检查是否达到最大水平距离（ROS2坐标系：z轴向上，x和y构成水平面）
      double horizontal_distance = sqrt(position_world.x()*position_world.x() + position_world.y()*position_world.y());
      if (horizontal_distance >= max_distance) {
        break;
      }
    }

    // 发射原点（世界坐标系）
    Eigen::Vector3d position_cam_origin = trajectory_predictor_->calculateIdealTrajectoryPoint(0, BULLET_V0);
    Eigen::Vector3d launch_origin_world;
    launch_origin_world.x() = position_cam_origin.z();
    launch_origin_world.y() = -position_cam_origin.x();
    launch_origin_world.z() = -position_cam_origin.y();
    launch_origin_world = R_cam_to_world * launch_origin_world;

    // 添加起点标记
    visualization_msgs::msg::Marker start_marker;
    start_marker.header = trajectory_marker_.header;
    start_marker.ns = "trajectory_start";
    start_marker.type = visualization_msgs::msg::Marker::SPHERE;
    start_marker.action = visualization_msgs::msg::Marker::ADD;
    start_marker.scale.x = start_marker.scale.y = start_marker.scale.z = 0.02;
    start_marker.color.a = 1.0;
    start_marker.color.g = 1.0;
    start_marker.pose.position.x = launch_origin_world.x();
    start_marker.pose.position.y = launch_origin_world.y();
    start_marker.pose.position.z = launch_origin_world.z();
    start_marker.pose.orientation.w = 1.0;
    marker_array.markers.push_back(start_marker);

    marker_array.markers.push_back(trajectory_marker_);
  }

  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker linear_v_marker_;
  visualization_msgs::msg::Marker angular_v_marker_;
  visualization_msgs::msg::Marker armor_marker_;
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
  auto ros2_publisher = std::make_shared<ROS2Publisher>();

  // 初始化实际硬件
  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  // 初始化自瞄组件
  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  // 初始化弹丸检测相关组件
  aimer::CoordConverter converter(config_path);
  aimer::aim::DoReproj do_reproj(
    converter.get_f_cv_mat_ref().clone(),
    converter.get_rot_ic_sup_cv_mat_ref().clone()
  );
  aimer::aim::DetectBullet bullet_detector(do_reproj);
  
  // 初始化弹道轨迹预测器
  ProjectileTrajectoryPredictor trajectory_predictor(&converter);

  // 设置ROS2Publisher的弹道预测器
  ros2_publisher->set_trajectory_predictor(&trajectory_predictor);

  std::mutex plan_mutex;
  auto_aim::Plan latest_plan = {};

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  std::atomic<bool> quit = false;
  std::thread ros2_spin_thread([&]() {
    while (!quit) {
      rclcpp::spin_some(ros2_publisher);
      std::this_thread::sleep_for(1ms);
    }
  });
  
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      auto target = target_queue.front();
      auto gs = gimbal.state();
      
      // 将optional<Target>转换为list<Target>
      std::list<auto_aim::Target> target_list;
      if (target.has_value()) {
        target_list.push_back(target.value());
      }
      
      auto plan = planner.plan(target, gs.bullet_speed);

      {
        std::lock_guard<std::mutex> lock(plan_mutex);
        latest_plan = plan;
      }

      gimbal.send(
        plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
        plan.pitch_acc);

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

  while (!exiter.exit()) {
    // 从实际相机读取图像
    camera.read(img, t);
    auto q = gimbal.q(t);
    auto gs = gimbal.state();
    auto mode = gimbal.mode();

    solver.set_R_gimbal2world(q);
    
    Eigen::Vector3d t_camera2world = solver.R_gimbal2world().transpose() * Eigen::Vector3d(0.145, 0, 0.07);
    ros2_publisher->publish_tf(q, t_camera2world);
    
    // 发布云台反馈消息
    uint8_t mode_value = 0;
    switch (mode) {
      case io::GimbalMode::IDLE: mode_value = 0; break;
      case io::GimbalMode::AUTO_AIM: mode_value = 1; break;
      case io::GimbalMode::SMALL_BUFF: mode_value = 2; break;
      case io::GimbalMode::BIG_BUFF: mode_value = 3; break;
    }
    ros2_publisher->publish_gimbal_feedback(gs, mode_value, q);
    
    // 自瞄核心逻辑
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

      Eigen::Vector3d position(target.ekf_x()[0], target.ekf_x()[2], target.ekf_x()[4]);
      Eigen::Vector3d velocity(target.ekf_x()[1], target.ekf_x()[3], target.ekf_x()[5]);
      Eigen::Vector3d angular_velocity(0, 0, target.ekf_x()[7]);
      
      std::vector<Eigen::Vector3d> armor_positions;
      std::vector<double> armor_yaws;
      for (const auto & xyza : armor_xyza_list) {
        armor_positions.push_back(xyza.head(3));
        armor_yaws.push_back(xyza[3]);
      }
      
      ros2_publisher->publish_marker(true, position, velocity, angular_velocity, armor_positions, armor_yaws, q);
    } else {
      ros2_publisher->publish_marker(false, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), {}, {}, q);
    }

    /// 弹丸检测逻辑
    // 设置当前时间和图像
    double current_time = tools::delta_time(t, t0);
    converter.set_img_t(current_time);
    converter.update(img.clone(), q, current_time);
    
    // 应用延迟补偿
    double compensated_time = tools::delta_time(t, t0) + SYSTEM_LATENCY;
    
    // 检测弹丸
    std::vector<aimer::aim::ImageBullet> detected_bullets = 
      bullet_detector.process_new_frame(img.clone(), q);
    
    // 处理检测到的弹丸
    std::vector<aimer::aim::ImageBullet> filtered_bullets;
    std::vector<std::tuple<aimer::aim::ImageBullet, int, double>> bullets_to_visualize;
    
    if (!detected_bullets.empty()) {
      tools::logger()->info("Detected {} bullets", detected_bullets.size());
      
      // 为每个检测到的弹丸计算速度并过滤
      for (const auto& bullet : detected_bullets) {
        // 通过弹丸大小估算距离
        double focal_length = converter.get_f_cv_mat_ref().at<double>(0, 0);
        double estimated_distance = (ACTUAL_BULLET_RADIUS * focal_length) / bullet.radius;
        
        // 过滤：弹丸距离
        if (estimated_distance < MIN_BULLET_DISTANCE || estimated_distance > MAX_BULLET_DISTANCE) {
          continue;
        }
        
        // 基础过滤：弹丸亮度和颜色
        if (bullet.center.x >= 0 && bullet.center.y >= 0 && 
            bullet.center.x < img.cols && bullet.center.y < img.rows) {
          cv::Vec3b pixel = img.at<cv::Vec3b>(bullet.center);
          double brightness = pixel[2];
          
          if (brightness < BULLET_BRIGHTNESS_THRESHOLD) {
            continue;
          }
        }
        
        // 为弹丸创建虚拟ID
        int matched_bullet_id = -1;
        double min_distance = BULLET_MATCH_DISTANCE_THRESHOLD;
        
        // 寻找最近的历史弹丸
        for (const auto& entry : bullet_histories) {
          double distance_pixels = cv::norm(bullet.center - entry.second.last_position);
          if (distance_pixels < min_distance) {
            min_distance = distance_pixels;
            matched_bullet_id = entry.first;
          }
        }
        
        // 如果找到匹配的历史弹丸，且距离在阈值内
        if (matched_bullet_id != -1 && min_distance < BULLET_MATCH_DISTANCE_THRESHOLD) {
          auto& history = bullet_histories[matched_bullet_id];
          history.consecutive_detections++;
          
          // 计算时间差和距离差
          double time_diff = tools::delta_time(t, t0) - history.last_time;
          if (time_diff > 0 && time_diff < BULLET_MAX_TIME_DIFF) {
            double distance_pixels = cv::norm(bullet.center - history.last_position);
            
            // 使用估算的距离计算实际移动距离
            double pixel_to_meter_ratio = estimated_distance / converter.get_img_ref().cols;
            double distance_meters = distance_pixels * pixel_to_meter_ratio;
            
            // 计算速度
            double speed = distance_meters / time_diff;
            history.estimated_speed = speed;
            
            // 计算轨迹成本
            history.trajectory_cost = trajectory_predictor.calculateTrajectoryCost(bullet.center, compensated_time, speed);
            
            bool speed_valid = (speed >= MIN_BULLET_SPEED - 2.0 && speed <= MAX_BULLET_SPEED + 10.0);
            
            if (speed_valid || history.consecutive_detections < 3) {
              filtered_bullets.push_back(bullet);
              bullets_to_visualize.emplace_back(bullet, matched_bullet_id, speed);
            }
          } else {
            filtered_bullets.push_back(bullet);
            bullets_to_visualize.emplace_back(bullet, matched_bullet_id, 0.0);
          }
          
          // 更新历史记录
          history.last_position = bullet.center;
          history.last_time = tools::delta_time(t, t0);
          history.estimated_distance = estimated_distance;
        } else {
          // 新弹丸，创建虚拟ID
          int new_bullet_id = next_bullet_id++;
          BulletHistory new_history(bullet.center, tools::delta_time(t, t0), estimated_distance);
          
          // 计算初始速度
          new_history.estimated_speed = BULLET_V0;
          
          // 计算轨迹成本
          new_history.trajectory_cost = trajectory_predictor.calculateTrajectoryCost(bullet.center, compensated_time, new_history.estimated_speed);
          
          bullet_histories[new_bullet_id] = new_history;
          filtered_bullets.push_back(bullet);
          
          bullets_to_visualize.emplace_back(bullet, new_bullet_id, new_history.estimated_speed);
        }
      }
      
      // 限制子弹数量：保留轨迹成本最低的子弹
      if (bullet_histories.size() > MAX_BULLET_COUNT) {
        std::vector<std::pair<int, double>> bullet_costs;
        for (const auto& entry : bullet_histories) {
          bullet_costs.emplace_back(entry.first, entry.second.trajectory_cost);
        }
        
        std::sort(bullet_costs.begin(), bullet_costs.end(), 
                  [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
                    return a.second < b.second;
                  });
        
        std::vector<int> to_remove;
        for (size_t i = MAX_BULLET_COUNT; i < bullet_costs.size(); ++i) {
          to_remove.push_back(bullet_costs[i].first);
        }
        
        for (int id : to_remove) {
          bullet_histories.erase(id);
        }
      }
      
      // 可视化保留在历史记录中的弹丸
      for (const auto& bullet_info : bullets_to_visualize) {
        const aimer::aim::ImageBullet& bullet = std::get<0>(bullet_info);
        int bullet_id = std::get<1>(bullet_info);
        double speed = std::get<2>(bullet_info);
        
        // 检查该弹丸是否在历史记录中
        if (bullet_histories.find(bullet_id) != bullet_histories.end()) {
          double font_scale = std::max(MIN_FONT_SIZE, std::min(MAX_FONT_SIZE, bullet.radius * FONT_SIZE_SCALE));
          
          // 绘制检测到的弹丸
          cv::circle(img, bullet.center, bullet.radius, cv::Scalar(0, 255, 0), 2);
          
          // 绘制文字，使用自适应字体大小
          std::string text;
          if (speed > 0) {
            text = fmt::format("Bullet {}: {:.1f}m/s", bullet_id, speed);
          } else {
            text = fmt::format("Bullet {}", bullet_id);
          }
          cv::putText(
            img,
            text,
            {static_cast<int>(bullet.center.x + bullet.radius + 5), static_cast<int>(bullet.center.y - bullet.radius - 5)},
            cv::FONT_HERSHEY_SIMPLEX,
            font_scale,
            cv::Scalar(0, 255, 0),
            1,
            cv::LINE_AA
          );
        }
      }
    }
    
    // 清理过期的弹丸历史记录
    std::vector<int> to_remove;
    for (const auto& entry : bullet_histories) {
      if (tools::delta_time(t, t0) - entry.second.last_time > 1.0) {
        to_remove.push_back(entry.first);
      }
    }
    for (int id : to_remove) {
      bullet_histories.erase(id);
    }

    {
      std::lock_guard<std::mutex> lock(plan_mutex);
      auto text = fmt::format(
        "yaw: {:.2f} pitch: {:.2f} fire: {}", latest_plan.yaw, latest_plan.pitch, latest_plan.fire);
      cv::putText(img, text, {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);
    }

    auto delay_text = fmt::format(
      "delay: {:.1f} ms", tools::delta_time(std::chrono::steady_clock::now(), t) * 1000.0);
    cv::putText(img, delay_text, {10, 60}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);

    // 绘制理想弹道的抛物线
    trajectory_predictor.drawIdealTrajectory(img, BULLET_V0, -1.0, 200);
    
    ros2_publisher->publish_image(img, "camera/image_raw");

    // 添加时间延迟，使处理速度接近正常速度
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  if (ros2_spin_thread.joinable()) ros2_spin_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
  rclcpp::shutdown();

  return 0;
}