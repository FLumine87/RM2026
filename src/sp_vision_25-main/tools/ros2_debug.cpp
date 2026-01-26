#include "ros2_debug.hpp"

#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace tools
{

ROS2DebugPublisher::ROS2DebugPublisher()
: Node("sp_vision_debug_publisher")
{
  robot_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot_pose", 10);
  target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 10);
  target_info_pub_ = this->create_publisher<auto_aim_debug_interfaces::msg::DebugTarget>("/target_info", 10);
  debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/debug_image", 10);
  auto_aim_armors_pub_ = this->create_publisher<auto_aim_debug_interfaces::msg::DebugArmors>("/auto_aim", 10);
  yolo_detection_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/yolo_detection", 10);

  RCLCPP_INFO(this->get_logger(), "ROS2 Debug Publisher initialized.");
}

void ROS2DebugPublisher::publishRobotPose(double yaw, double pitch)
{
  auto msg = geometry_msgs::msg::PoseStamped();
  msg.header.stamp = this->now();
  msg.header.frame_id = "gimbal";

  msg.pose.position.x = 0.0;
  msg.pose.position.y = 0.0;
  msg.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, -pitch, yaw);
  msg.pose.orientation = tf2::toMsg(q);

  robot_pose_pub_->publish(msg);
}

void ROS2DebugPublisher::publishTargetPose(const std::optional<auto_aim::Target> & target)
{
  auto msg = geometry_msgs::msg::PoseStamped();
  msg.header.stamp = this->now();
  msg.header.frame_id = "world";

  if (target.has_value()) {
    auto x = target->ekf_x();
    msg.pose.position.x = x[0];
    msg.pose.position.y = x[2];
    msg.pose.position.z = x[4];

    double yaw = x[6];
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    msg.pose.orientation = tf2::toMsg(q);
  } else {
    msg.pose.position.x = 0.0;
    msg.pose.position.y = 0.0;
    msg.pose.position.z = 0.0;
    msg.pose.orientation.w = 1.0;
  }

  target_pose_pub_->publish(msg);
}

void ROS2DebugPublisher::publishTargetInfo(const std::optional<auto_aim::Target> & target)
{
  auto msg = auto_aim_debug_interfaces::msg::DebugTarget();
  msg.header.stamp = this->now();
  msg.header.frame_id = "world";

  if (target.has_value()) {
    auto x = target->ekf_x();
    msg.id = auto_aim::ARMOR_NAMES[target->name];
    msg.type = auto_aim::ARMOR_TYPES[target->armor_type];
    msg.priority = target->priority;
    msg.jumped = target->jumped;
    msg.last_id = target->last_id;
    
    msg.position_x = x[0];
    msg.position_y = x[2];
    msg.position_z = x[4];
    msg.velocity_x = x[1];
    msg.velocity_y = x[3];
    msg.velocity_z = x[5];
    msg.yaw = x[6];
    msg.yaw_rate = x[7];
    msg.radius = x[8];
    msg.l = x[9];
    msg.h = x[10];
    
    auto armor_xyza_list = target->armor_xyza_list();
    msg.armor_count = armor_xyza_list.size();
  } else {
    msg.id = "none";
    msg.type = "none";
    msg.priority = 0;
    msg.jumped = false;
    msg.last_id = 0;
    msg.position_x = 0.0;
    msg.position_y = 0.0;
    msg.position_z = 0.0;
    msg.velocity_x = 0.0;
    msg.velocity_y = 0.0;
    msg.velocity_z = 0.0;
    msg.yaw = 0.0;
    msg.yaw_rate = 0.0;
    msg.radius = 0.0;
    msg.l = 0.0;
    msg.h = 0.0;
    msg.armor_count = 0;
  }

  target_info_pub_->publish(msg);
}

void ROS2DebugPublisher::publishDebugImage(const cv::Mat & img)
{
  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
  msg->header.stamp = this->now();
  msg->header.frame_id = "camera";

  debug_image_pub_->publish(*msg);
}

void ROS2DebugPublisher::publishAutoAimArmors(const std::list<auto_aim::Armor> & armors)
{
  auto msg = auto_aim_debug_interfaces::msg::DebugArmors();

  msg.data.resize(armors.size());
  size_t i = 0;
  for (const auto & armor : armors) {
    auto & armor_msg = msg.data[i];
    armor_msg.header.stamp = this->now();
    armor_msg.header.frame_id = "camera";
    
    armor_msg.color = auto_aim::COLORS[armor.color];
    armor_msg.name = auto_aim::ARMOR_NAMES[armor.name];
    armor_msg.type = auto_aim::ARMOR_TYPES[armor.type];
    armor_msg.confidence = armor.confidence;

    armor_msg.center_x = armor.center.x;
    armor_msg.center_y = armor.center.y;
    armor_msg.center_norm_x = armor.center_norm.x;
    armor_msg.center_norm_y = armor.center_norm.y;

    armor_msg.xyz_in_gimbal_x = armor.xyz_in_gimbal.x();
    armor_msg.xyz_in_gimbal_y = armor.xyz_in_gimbal.y();
    armor_msg.xyz_in_gimbal_z = armor.xyz_in_gimbal.z();
    armor_msg.xyz_in_world_x = armor.xyz_in_world.x();
    armor_msg.xyz_in_world_y = armor.xyz_in_world.y();
    armor_msg.xyz_in_world_z = armor.xyz_in_world.z();

    armor_msg.ypr_in_gimbal_x = armor.ypr_in_gimbal.x();
    armor_msg.ypr_in_gimbal_y = armor.ypr_in_gimbal.y();
    armor_msg.ypr_in_gimbal_z = armor.ypr_in_gimbal.z();
    armor_msg.ypr_in_world_x = armor.ypr_in_world.x();
    armor_msg.ypr_in_world_y = armor.ypr_in_world.y();
    armor_msg.ypr_in_world_z = armor.ypr_in_world.z();

    armor_msg.yaw_raw = armor.yaw_raw;

    if (armor.points.size() >= 4) {
      armor_msg.points_0_x = armor.points[0].x;
      armor_msg.points_0_y = armor.points[0].y;
      armor_msg.points_1_x = armor.points[1].x;
      armor_msg.points_1_y = armor.points[1].y;
      armor_msg.points_2_x = armor.points[2].x;
      armor_msg.points_2_y = armor.points[2].y;
      armor_msg.points_3_x = armor.points[3].x;
      armor_msg.points_3_y = armor.points[3].y;
    }

    armor_msg.ratio = armor.ratio;
    armor_msg.side_ratio = armor.side_ratio;
    armor_msg.rectangular_error = armor.rectangular_error;

    i++;
  }

  auto_aim_armors_pub_->publish(msg);
}

void ROS2DebugPublisher::publishYOLODetectionImage(const cv::Mat & img)
{
  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
  msg->header.stamp = this->now();
  msg->header.frame_id = "camera";

  yolo_detection_pub_->publish(*msg);
}

}  // namespace tools
