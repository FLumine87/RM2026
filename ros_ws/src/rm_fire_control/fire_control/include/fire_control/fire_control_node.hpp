#ifndef FIRE_CONTROL_NODE_HPP_
#define FIRE_CONTROL_NODE_HPP_

// ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

// STD
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <mutex>
#include <optional> 

#include "fire_control/solver.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "fire_control_interfaces/msg/gimbal_cmd.hpp"

namespace rm_fire_control
{
class FireControlNode : public rclcpp::Node
{
public:
  explicit FireControlNode(const rclcpp::NodeOptions & options);
  
private:
  // Callbacks
  void ControlTimerCallback();
  void TargetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg);

  // TF2相关
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  std::shared_ptr<tf2_ros::MessageFilter<auto_aim_interfaces::msg::Target>> tf2_filter_;
  std::string target_frame_;

  // 订阅/发布
  message_filters::Subscriber<auto_aim_interfaces::msg::Target> target_sub_;
  rclcpp::Publisher<fire_control_interfaces::msg::GimbalCmd>::SharedPtr gimbal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  
  // 定时器
  rclcpp::TimerBase::SharedPtr control_timer_;

  // 数据共享
  std::mutex target_mutex_;
  std::shared_ptr<auto_aim_interfaces::msg::Target> latest_target_;

  // 可视化标记
  visualization_msgs::msg::Marker aiming_point_;

  // 弹道解算器
  std::unique_ptr<Solver> solver_;
};

} // namespace rm_fire_control

#endif