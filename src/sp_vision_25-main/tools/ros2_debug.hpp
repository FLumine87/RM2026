#ifndef TOOLS__ROS2_DEBUG_HPP
#define TOOLS__ROS2_DEBUG_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/target.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "auto_aim_debug_interfaces/msg/debug_armor.hpp"
#include "auto_aim_debug_interfaces/msg/debug_armors.hpp"
#include "auto_aim_debug_interfaces/msg/debug_target.hpp"

namespace tools
{

class ROS2DebugPublisher : public rclcpp::Node
{
public:
  ROS2DebugPublisher();

  void publishRobotPose(double yaw, double pitch);

  void publishTargetPose(const std::optional<auto_aim::Target> & target);

  void publishTargetInfo(const std::optional<auto_aim::Target> & target);

  void publishDebugImage(const cv::Mat & img);

  void publishAutoAimArmors(const std::list<auto_aim::Armor> & armors);

  void publishYOLODetectionImage(const cv::Mat & img);

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<auto_aim_debug_interfaces::msg::DebugTarget>::SharedPtr target_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Publisher<auto_aim_debug_interfaces::msg::DebugArmors>::SharedPtr auto_aim_armors_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr yolo_detection_pub_;
};

}  // namespace tools

#endif
