
//  C++ system
#include <memory>
#include <string>
#include <vector>
#include <optional> 

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/convert.h>
#include "fire_control/fire_control_node.hpp"


namespace rm_fire_control
{
FireControlNode::FireControlNode(const rclcpp::NodeOptions & options) 
:Node("fire_control",options), solver_(nullptr)
{
  RCLCPP_INFO(this->get_logger(), "Starting FireControlNode!");

  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);

  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  
  //  target subscribe and fliter
  target_sub_.subscribe(this, "/tracker/target", rmw_qos_profile_sensor_data);
  target_frame_ = this->declare_parameter("target_frame", "odom");
  tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<auto_aim_interfaces::msg::Target>>(target_sub_,
                                             *tf2_buffer_,
                                             target_frame_,
                                             10,
                                             this->get_node_logging_interface(),
                                             this->get_node_clock_interface(),
                                             std::chrono::duration<int>(1));
  // Register a callback with tf2_ros::MessageFilter to be called when
  // transforms are available
  tf2_filter_->registerCallback(&FireControlNode::TargetCallback, this);

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);

  gimbal_pub_ = this->create_publisher<fire_control_interfaces::msg::GimbalCmd>(
    "fire_control/cmd_gimbal",rclcpp::SensorDataQoS());
    
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(4),
    std::bind(&FireControlNode::ControlTimerCallback, this));

  //debug_mode_ = this->declare_parameter<bool>("debug_mode", false); //之后用于发布markers
  RCLCPP_INFO(this->get_logger(), "FireControlNode initialization completed!"); 

  aiming_point_.header.frame_id = "odom";
  aiming_point_.ns = "aiming_point";
  aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
  aiming_point_.action = visualization_msgs::msg::Marker::ADD;
  aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
  aiming_point_.color.r = 1.0;
  aiming_point_.color.g = 1.0;
  aiming_point_.color.b = 1.0;
  aiming_point_.color.a = 1.0;
  aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);
}

void FireControlNode::ControlTimerCallback()
{
  //init solver_
  if (solver_ == nullptr) {
    solver_ = std::make_unique<Solver>(weak_from_this());
  }


  fire_control_interfaces::msg::GimbalCmd control_msg;
  control_msg.tracking = false;
  control_msg.id = "";
  control_msg.armors_num = 0;
  control_msg.yaw_diff = 0.0;
  control_msg.pitch_diff = 0.0;
  control_msg.distance = -1.0;
  control_msg.pitch = 0.0;
  control_msg.yaw = 0.0;
  control_msg.fire_advice = false;

  std::shared_ptr<auto_aim_interfaces::msg::Target> current_target;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    current_target = latest_target_;
  }

  if (!current_target) {
    gimbal_pub_->publish(control_msg);
    return;
  }

  if(current_target->header.frame_id.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Empty target frame_id");
    gimbal_pub_->publish(control_msg); 
    return;
  }


  const auto target_time = tf2_ros::fromMsg(current_target->header.stamp);


  if (!tf2_buffer_->canTransform(  
        target_frame_,
        current_target->header.frame_id,  
        target_time))
  {  
        RCLCPP_WARN(this->get_logger(), "Required transform is not available");
        gimbal_pub_->publish(control_msg);  
        return;  
  }


  if (current_target->tracking) {  
    try {  
      control_msg = solver_->Solve(*current_target, tf2_buffer_);  
    } catch (const std::runtime_error &e) {  
      RCLCPP_ERROR(this->get_logger(), "Runtime error in solver: %s", e.what());  
    } catch (const std::exception &e) {  
      RCLCPP_ERROR(this->get_logger(), "Exception in solver: %s", e.what());  
    } catch (...) {  
      RCLCPP_ERROR(this->get_logger(), "Unknown error in solver!");  
    }
  }
  // 发布控制指令
  gimbal_pub_->publish(control_msg);

  // 更新并发布瞄准点
  if (abs(control_msg.aim_x) > 0.01) {
    aiming_point_.header.stamp = this->now();
    aiming_point_.pose.position.x = control_msg.aim_x;
    aiming_point_.pose.position.y = control_msg.aim_y;
    aiming_point_.pose.position.z = control_msg.aim_z;
    marker_pub_->publish(aiming_point_);
  }
  
}

void FireControlNode::TargetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg)  
{  
  std::lock_guard<std::mutex> lock(target_mutex_);
  latest_target_ = std::make_shared<auto_aim_interfaces::msg::Target>(*msg);
}





//no timer send
// void FireControlNode::TargetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg)  
// {  
//   //init solver_
//   if (solver_ == nullptr) {
//     solver_ = std::make_unique<Solver>(weak_from_this());
//   }

//   // RCLCPP_INFO(this->get_logger(),   
//   //   "Target received - frame_id: %s, tracking: %d, stamp: %.3f",  
//   //   msg->header.frame_id.c_str(),  
//   //   msg->tracking,  
//   //   rclcpp::Time(msg->header.stamp).seconds());  



//   fire_control_interfaces::msg::GimbalCmd control_msg; 

//   control_msg.tracking = msg->tracking;
//   control_msg.id = msg->id;
//   control_msg.armors_num = msg->armors_num;

//   control_msg.yaw_diff = 0.0;
//   control_msg.pitch_diff = 0.0;
//   control_msg.distance = -1.0;
//   control_msg.pitch = 0.0;
//   control_msg.yaw = 0.0;
//   control_msg.fire_advice = false;

//   if(solver_ == nullptr)
//   {
//     return;
//   }

//   if (!msg) 
//   { 
//     gimbal_pub_->publish(control_msg);
//     return;  
//   }  

//   if(msg->header.frame_id.empty())
//   {
//     RCLCPP_WARN(this->get_logger(), "Empty target frame_id");
//     gimbal_pub_->publish(control_msg); 
//     return;
//   }

//   const auto target_time = tf2_ros::fromMsg(msg->header.stamp);

//   if (!tf2_buffer_->canTransform(  
//         target_frame_,
//         msg->header.frame_id,  
//         target_time)) 
//   {  
//         RCLCPP_WARN(this->get_logger(), "Required transform is not available");
//         gimbal_pub_->publish(control_msg);  
//         return;  
//   }

//   if (msg->tracking) {  
//     try {  
//       control_msg = solver_->Solve(*msg, tf2_buffer_);  
//     } catch (const std::runtime_error &e) {  
//       RCLCPP_ERROR(this->get_logger(), "Runtime error in solver: %s", e.what());  
//       control_msg.yaw_diff = 0.0;
//       control_msg.pitch_diff = 0.0;
//       control_msg.distance = -1.0;
//       control_msg.pitch = 0.0;
//       control_msg.yaw = 0.0;
//       control_msg.fire_advice = false;
//     } catch (const std::exception &e) {  
//       RCLCPP_ERROR(this->get_logger(), "Exception in solver: %s", e.what());  
//       control_msg.yaw_diff = 0.0;
//       control_msg.pitch_diff = 0.0;
//       control_msg.distance = -1.0;
//       control_msg.pitch = 0.0;
//       control_msg.yaw = 0.0;
//       control_msg.fire_advice = false;
//     } catch (...) {  
//       RCLCPP_ERROR(this->get_logger(), "Unknown error in solver!");  
//     }  
//   }

//   if (abs(control_msg.aim_x) > 0.01) {
//             aiming_point_.header.stamp = this->now();
//             aiming_point_.pose.position.x = control_msg.aim_x;
//             aiming_point_.pose.position.y = control_msg.aim_y;
//             aiming_point_.pose.position.z = control_msg.aim_z;
//             marker_pub_->publish(aiming_point_);
//           }

//   gimbal_pub_->publish(control_msg);
// }  

}    //namespace rm_fire_control

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rm_fire_control::FireControlNode)
