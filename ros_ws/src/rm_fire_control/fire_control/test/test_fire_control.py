#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from auto_aim_interfaces.msg import Target
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.target_pub = self.create_publisher(Target, '/tracker/target', 10)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # 发布静态tf
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'odom'
        static_transform.child_frame_id = 'gimbal_link'
        static_transform.transform.translation.x = 0.0
        static_transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(static_transform)
        
        # 定时发布Target
        self.timer = self.create_timer(0.5, self.publish_target)
        
    def publish_target(self):
        msg = Target()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.tracking = True
        msg.id = "2";  
        msg.armors_num = 1; 
        msg.position.x = 1.0
        msg.position.y = 1.0
        msg.position.z = 0.0
        msg.velocity.x = 0.1
        msg.velocity.y = 0.0 
        msg.velocity.z = 0.0 
        msg.yaw = 0.0
        msg.v_yaw = 0.1
        msg.radius_1 = 0.5 
        msg.radius_2 = 0.5  
        msg.dz = 0.0
        self.target_pub.publish(msg)

def main():
    rclpy.init()
    node = TestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()