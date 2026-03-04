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

#ifdef ROS2_INCLUDE_PATH
#include <auto_aim_interfaces/msg/armor.hpp>
#include <auto_aim_interfaces/msg/armors.hpp>
#include <auto_aim_interfaces/msg/target.hpp>
#else
#include "auto_aim_interfaces/msg/armor.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#endif

#include "tasks/bullet_detector/aim_corrector.hpp"

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
const double FONT_SIZE_SCALE = 0.2; // 字体大小缩放因子
const double MIN_FONT_SIZE = 0.3; // 最小字体大小
const double MAX_FONT_SIZE = 0.8; // 最大字体大小

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
        // 理想弹道：考虑重力，从发射原点开始
        double t = time;
        
        // 发射朝向偏移（ypr格式）
        double yaw = LAUNCH_DIRECTION_OFFSET.x();
        double pitch = LAUNCH_DIRECTION_OFFSET.y();
        
        // 计算理想位置（相机坐标系：z向前，x向右，y向下）
        Eigen::Vector3d position;
        
        // 速度分量计算
        double vx = initial_speed * sin(yaw) * cos(pitch);      // 水平向右速度
        double vy = initial_speed * sin(pitch);                 // 垂直向下速度（相机坐标系y轴向下为正）
        double vz = initial_speed * cos(pitch) * cos(yaw);      // 向前速度
        
        // 位置计算（考虑重力）
        position.x() = vx * t;
        position.y() = vy * t + 0.5 * GRAVITY * t * t;  // 重力向下的位移（相机坐标系y轴向下为正）
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
            // 最大飞行时间：当弹丸落地时（y=0）
            double pitch = LAUNCH_DIRECTION_OFFSET.y();
            double vy_initial = initial_speed * sin(pitch);
            
            // 解方程：vy_initial * t + 0.5 * GRAVITY * t^2 = 0
            if (vy_initial > 0) {
                // 有初始向上速度，先上升后下落
                max_time = (2 * vy_initial) / GRAVITY;
            } else {
                // 直接下落
                max_time = sqrt((2 * std::abs(LAUNCH_ORIGIN_OFFSET.y())) / GRAVITY);
            }
            
            // 确保最大时间合理
            max_time = std::max(0.5, std::min(max_time, 5.0)); // 限制在0.5-5秒之间
        }
        std::vector<cv::Point2f> trajectory_points;
        
        // 计算弹道上的多个点
        double time_step = max_time / num_points;
        for (int i = 0; i <= num_points; ++i) {
            double time = i * time_step;
            Eigen::Vector3d position_cam = calculateIdealTrajectoryPoint(time, initial_speed);
            
            // 使用正确的相机投影方法（对于相机坐标系中的点）
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
                // 根据距离计算线宽（z越大，距离越远，线越细）
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
    : Node("bullet_detector_rosbc_publisher")
    , trajectory_predictor_(trajectory_predictor)
    , tf_publisher_(this->create_publisher<tf2_msgs::msg::TFMessage>("tf", 10))
    , image_publisher_(this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10))
    , armor_msg_publisher_(this->create_publisher<auto_aim_interfaces::msg::Armors>("armor_msg", 10))
    , target_msg_publisher_(this->create_publisher<auto_aim_interfaces::msg::Target>("target_msg", 10))
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

  // 弹道轨迹marker
  visualization_msgs::msg::Marker trajectory_marker_;

  void publish_marker(
    bool tracking, const Eigen::Vector3d & position, const Eigen::Vector3d & velocity, 
    const Eigen::Vector3d & angular_velocity, const std::vector<Eigen::Vector3d> & armor_positions,
    const std::vector<double> & armor_yaws, const Eigen::Quaterniond & q_gimbal)
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

      // 发布弹道轨迹marker
      publish_trajectory_marker(now, marker_array, q_gimbal);

      marker_array.markers.push_back(position_marker_);
      marker_array.markers.push_back(linear_v_marker_);
      marker_array.markers.push_back(angular_v_marker_);
    } else {
      position_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
      linear_v_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
      angular_v_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
      armor_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
      trajectory_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
      marker_array.markers.push_back(armor_marker_);
      marker_array.markers.push_back(trajectory_marker_);
    }

    marker_pub_->publish(marker_array);
  }

  // 发布弹道轨迹marker
  void publish_trajectory_marker(const rclcpp::Time & now, visualization_msgs::msg::MarkerArray & marker_array, const Eigen::Quaterniond & q_gimbal)
  {
    // 初始化轨迹marker
    trajectory_marker_.header.stamp = now;
    trajectory_marker_.header.frame_id = "world";
    trajectory_marker_.ns = "trajectory";
    trajectory_marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trajectory_marker_.action = visualization_msgs::msg::Marker::ADD;
    trajectory_marker_.scale.x = 0.02; // 线宽
    trajectory_marker_.color.a = 1.0;
    trajectory_marker_.color.r = 1.0;
    trajectory_marker_.color.g = 0.0;
    trajectory_marker_.color.b = 1.0;
    trajectory_marker_.points.clear();

    // 相机坐标系到世界坐标系的转换
    Eigen::Matrix3d R_cam_to_world = q_gimbal.toRotationMatrix();
    
    // 最大距离
    const double max_distance = MAX_BULLET_DISTANCE; // 20米
    
    // 计算弹道轨迹点（抛物线）
    int num_points = 200; // 轨迹点数量
    double total_time = max_distance / BULLET_V0; // 总飞行时间
    double time_step = total_time / num_points;
    
    for (int i = 0; i <= num_points; ++i) {
      double t = i * time_step;
      
      // 计算弹道位置（相机坐标系）
      Eigen::Vector3d position_cam;
      
      if (trajectory_predictor_) {
        // 使用trajectory_predictor_计算理想弹道
        position_cam = trajectory_predictor_->calculateIdealTrajectoryPoint(t, BULLET_V0);
      } else {
        // 备用方案：如果trajectory_predictor_不可用，使用传统计算方法
        double yaw_cam = LAUNCH_DIRECTION_OFFSET.x();
        double pitch_cam = LAUNCH_DIRECTION_OFFSET.y();
        
        // 速度分量（相机坐标系：z向前，x向右，y向下）
        double vx = BULLET_V0 * sin(yaw_cam);
        double vy = BULLET_V0 * sin(pitch_cam);
        double vz = BULLET_V0 * cos(pitch_cam) * cos(yaw_cam);
        
        // 位置计算（相机坐标系，重力向下）
        position_cam.x() = vx * t;
        position_cam.y() = vy * t + 0.5 * GRAVITY * t * t;  // 重力向下的位移（相机坐标系y轴向下为正，所以是加号）
        position_cam.z() = vz * t;
        
        // 添加发射原点偏移
        position_cam += LAUNCH_ORIGIN_OFFSET;
      }
      
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
      
      // 添加到marker点列表
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
    Eigen::Vector3d launch_origin_world = R_cam_to_world * LAUNCH_ORIGIN_OFFSET;
    
    // 添加轨迹起点标记（绿色）
    visualization_msgs::msg::Marker start_marker;
    start_marker.header = trajectory_marker_.header;
    start_marker.ns = "trajectory_start";
    start_marker.type = visualization_msgs::msg::Marker::SPHERE;
    start_marker.action = visualization_msgs::msg::Marker::ADD;
    start_marker.scale.x = start_marker.scale.y = start_marker.scale.z = 0.02; // 半径2cm
    start_marker.color.a = 1.0;
    start_marker.color.g = 1.0;
    start_marker.pose.position.x = launch_origin_world.x();
    start_marker.pose.position.y = launch_origin_world.y();
    start_marker.pose.position.z = launch_origin_world.z();
    start_marker.pose.orientation.w = 1.0;
    marker_array.markers.push_back(start_marker);

    // 添加轨迹线
    marker_array.markers.push_back(trajectory_marker_);
  }

  // 初始化轨迹marker
  void init_trajectory_marker()
  {
    trajectory_marker_.ns = "trajectory";
    trajectory_marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trajectory_marker_.scale.x = 0.02;
    trajectory_marker_.color.a = 1.0;
    trajectory_marker_.color.r = 1.0;
    trajectory_marker_.color.g = 0.0;
    trajectory_marker_.color.b = 1.0;
  }
  
  // 设置弹道预测器
  void set_trajectory_predictor(ProjectileTrajectoryPredictor* predictor)
  {
    trajectory_predictor_ = predictor;
  }

  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armor_msg_publisher_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_msg_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker linear_v_marker_;
  visualization_msgs::msg::Marker angular_v_marker_;
  visualization_msgs::msg::Marker armor_marker_;
  
  // 弹道预测器指针
  ProjectileTrajectoryPredictor* trajectory_predictor_;
};

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

  auto_aim::YOLO yolo(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);

  aimer::CoordConverter converter(config_path);
  aimer::aim::DoReproj do_reproj(
    converter.get_f_cv_mat_ref().clone(),
    converter.get_rot_ic_sup_cv_mat_ref().clone()
  );
  aimer::aim::DetectBullet bullet_detector(do_reproj);
  
  ProjectileTrajectoryPredictor trajectory_predictor(&converter);
  auto ros2_publisher = std::make_shared<ROS2Publisher>(&trajectory_predictor);

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  std::atomic<bool> quit = false;
  std::thread ros2_spin_thread([&]() { rclcpp::spin(ros2_publisher); });

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  int frame_count = 0;

  while (!exiter.exit()) {
    camera.read(img, t);
    auto q = gimbal.q(t);

    solver.set_R_gimbal2world(q);
    
    Eigen::Vector3d t_camera2world = solver.R_gimbal2world().transpose() * Eigen::Vector3d(0.145, 0, 0.07);
    ros2_publisher->publish_tf(q, t_camera2world);
    
    auto armors = yolo.detect(img);
    auto targets = tracker.track(armors, t);
    auto command = aimer.aim(targets, t, 27, false);

    auto finish = std::chrono::steady_clock::now();
    tools::logger()->info(
      "[{}] yolo: {:.1f}ms, tracker: {:.1f}ms, aimer: {:.1f}ms", frame_count,
      tools::delta_time(finish, t) * 1e3);

    tools::draw_text(
      img,
      fmt::format(
        "command is {},{:.2f},{:.2f},shoot:{}", command.control, command.yaw * 57.3,
        command.pitch * 57.3, command.shoot),
      {10, 60}, {154, 50, 205});

    Eigen::Quaternion<double> gimbal_q = {q.w(), q.x(), q.y(), q.z()};
    tools::draw_text(
      img,
      fmt::format(
        "gimbal yaw{:.2f}", (tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0) * 57.3)[0]),
      {10, 90}, {255, 255, 255});

    nlohmann::json data;

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

      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid) tools::draw_points(img, image_points, {0, 0, 255});

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

      data["residual_yaw"] = target.ekf().data.at("residual_yaw");
      data["residual_pitch"] = target.ekf().data.at("residual_pitch");
      data["residual_distance"] = target.ekf().data.at("residual_distance");
      data["residual_angle"] = target.ekf().data.at("residual_angle");
      data["nis"] = target.ekf().data.at("nis");
      data["nees"] = target.ekf().data.at("nees");
      data["nis_fail"] = target.ekf().data.at("nis_fail");
      data["nees_fail"] = target.ekf().data.at("nees_fail");
      data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");

      std::vector<auto_aim::Armor> armor_vector(armors.begin(), armors.end());
      ros2_publisher->publish_armor_msg(armor_vector);
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
      
      ros2_publisher->publish_marker(true, position, velocity, angular_velocity, armor_positions, armor_yaws, gimbal_q);
    } else {
      ros2_publisher->publish_marker(false, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), {}, {}, Eigen::Quaterniond::Identity());
    }

    double t_seconds = std::chrono::duration<double>(t - t0).count();
    converter.set_img_t(t_seconds);
    converter.update(img.clone(), gimbal_q, t_seconds);
    
    double compensated_time = t_seconds + SYSTEM_LATENCY;
    
    std::vector<aimer::aim::ImageBullet> detected_bullets = 
      bullet_detector.process_new_frame(img.clone(), gimbal_q);
    
    std::vector<aimer::aim::ImageBullet> filtered_bullets;
    std::vector<std::tuple<aimer::aim::ImageBullet, int, double>> bullets_to_visualize;
    
    if (!detected_bullets.empty()) {
      tools::logger()->info("[{}] Detected {} bullets", frame_count, detected_bullets.size());
      
      for (const auto& bullet : detected_bullets) {
        double focal_length = converter.get_f_cv_mat_ref().at<double>(0, 0);
        double estimated_distance = (ACTUAL_BULLET_RADIUS * focal_length) / bullet.radius;
        
        if (estimated_distance < MIN_BULLET_DISTANCE || estimated_distance > MAX_BULLET_DISTANCE) {
          tools::logger()->info("[{}] Bullet distance {:.2f} out of range (filtered out)", frame_count, estimated_distance);
          continue;
        }
        
        if (bullet.center.x >= 0 && bullet.center.y >= 0 && 
            bullet.center.x < img.cols && bullet.center.y < img.rows) {
          cv::Vec3b pixel = img.at<cv::Vec3b>(bullet.center);
          double brightness = pixel[2];
          
          if (brightness < BULLET_BRIGHTNESS_THRESHOLD) {
            tools::logger()->info("[{}] Bullet brightness {:.2f} too low (filtered out)", frame_count, brightness);
            continue;
          }
        }
        
        int matched_bullet_id = -1;
        double min_distance = BULLET_MATCH_DISTANCE_THRESHOLD;
        
        for (const auto& entry : bullet_histories) {
          double distance_pixels = cv::norm(bullet.center - entry.second.last_position);
          if (distance_pixels < min_distance) {
            min_distance = distance_pixels;
            matched_bullet_id = entry.first;
          }
        }
        
        double font_scale = std::max(MIN_FONT_SIZE, std::min(MAX_FONT_SIZE, bullet.radius * FONT_SIZE_SCALE));
        
        if (matched_bullet_id != -1 && min_distance < BULLET_MATCH_DISTANCE_THRESHOLD) {
          auto& history = bullet_histories[matched_bullet_id];
          history.consecutive_detections++;
          
          double time_diff = t_seconds - history.last_time;
          if (time_diff > 0 && time_diff < BULLET_MAX_TIME_DIFF) {
            double distance_pixels = cv::norm(bullet.center - history.last_position);
            double pixel_to_meter_ratio = estimated_distance / converter.get_img_ref().cols;
            double distance_meters = distance_pixels * pixel_to_meter_ratio;
            double speed = distance_meters / time_diff;
            history.estimated_speed = speed;
            
            history.trajectory_cost = trajectory_predictor.calculateTrajectoryCost(bullet.center, compensated_time, speed);
            
            bool speed_valid = (speed >= MIN_BULLET_SPEED - 2.0 && speed <= MAX_BULLET_SPEED + 10.0);
            
            if (speed_valid || history.consecutive_detections < 3) {
              filtered_bullets.push_back(bullet);
              if (speed_valid) {
                tools::logger()->info("[{}] Bullet {} speed: {:.2f} m/s, distance: {:.2f} m, cost: {:.2f} (filtered in)", frame_count, matched_bullet_id, speed, estimated_distance, history.trajectory_cost);
              } else {
                tools::logger()->info("[{}] Bullet {} speed: {:.2f} m/s (allowed for new bullet)", frame_count, matched_bullet_id, speed);
              }
              
              bullets_to_visualize.emplace_back(bullet, matched_bullet_id, speed);
            } else {
              tools::logger()->info("[{}] Bullet {} speed: {:.2f} m/s (filtered out)", frame_count, matched_bullet_id, speed);
            }
          } else {
            filtered_bullets.push_back(bullet);
            tools::logger()->info("[{}] Bullet {} time diff {:.3f}s (filtered in)", frame_count, matched_bullet_id, time_diff);
            
            bullets_to_visualize.emplace_back(bullet, matched_bullet_id, 0.0);
          }
          
          history.last_position = bullet.center;
          history.last_time = t_seconds;
          history.estimated_distance = estimated_distance;
        } else {
          int new_bullet_id = next_bullet_id++;
          BulletHistory new_history(bullet.center, t_seconds, estimated_distance);
          new_history.estimated_speed = BULLET_V0;
          new_history.trajectory_cost = trajectory_predictor.calculateTrajectoryCost(bullet.center, compensated_time, new_history.estimated_speed);
          
          bullet_histories[new_bullet_id] = new_history;
          filtered_bullets.push_back(bullet);
          
          tools::logger()->info("[{}] New bullet {} detected, distance: {:.2f} m, cost: {:.2f}", frame_count, new_bullet_id, estimated_distance, new_history.trajectory_cost);
          
          bullets_to_visualize.emplace_back(bullet, new_bullet_id, new_history.estimated_speed);
        }
      }
      
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
          tools::logger()->info("[{}] Removing high cost bullet {} (cost: {:.2f})\n", frame_count, id, bullet_histories[id].trajectory_cost);
          bullet_histories.erase(id);
        }
      }
      
      for (const auto& bullet_info : bullets_to_visualize) {
        const aimer::aim::ImageBullet& bullet = std::get<0>(bullet_info);
        int bullet_id = std::get<1>(bullet_info);
        double speed = std::get<2>(bullet_info);
        
        if (bullet_histories.find(bullet_id) != bullet_histories.end()) {
          double font_scale = std::max(MIN_FONT_SIZE, std::min(MAX_FONT_SIZE, bullet.radius * FONT_SIZE_SCALE));
          
          cv::circle(img, bullet.center, bullet.radius, cv::Scalar(0, 255, 0), 2);
          
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
      
      data["bullet_detected"] = !filtered_bullets.empty();
      data["bullet_count"] = filtered_bullets.size();
      data["original_bullet_count"] = detected_bullets.size();
      data["remaining_bullet_count"] = bullet_histories.size();
    } else {
      tools::logger()->info("[{}] No bullets detected", frame_count);
      data["bullet_detected"] = false;
      data["bullet_count"] = 0;
      data["original_bullet_count"] = 0;
      data["remaining_bullet_count"] = bullet_histories.size();
    }
    
    std::vector<int> to_remove;
    for (const auto& entry : bullet_histories) {
      if (t_seconds - entry.second.last_time > 1.0) {
        to_remove.push_back(entry.first);
      }
    }
    for (int id : to_remove) {
      tools::logger()->info("[{}] Removing expired bullet {}", frame_count, id);
      bullet_histories.erase(id);
    }

    plotter.plot(data);

    trajectory_predictor.drawIdealTrajectory(img, BULLET_V0, -1.0, 200);
    
    ros2_publisher->publish_image(img, "camera/image_raw");

    frame_count++;
  }

  quit = true;
  if (ros2_spin_thread.joinable()) ros2_spin_thread.join();
  rclcpp::shutdown();

  return 0;
}
