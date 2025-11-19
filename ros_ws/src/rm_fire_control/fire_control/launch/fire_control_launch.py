from launch import LaunchDescription  
from launch_ros.actions import Node  
from launch.actions import LogInfo  

def generate_launch_description():  
    return LaunchDescription([  
        LogInfo(msg="Starting Fire Control Node!"), 
        Node(  
            package="fire_control",  
            executable="fire_control_node",  
            name="fire_control",  
            output="screen",  
            parameters=[{"target_frame": "odom"}],  
            arguments=['--ros-args', '--log-level', 'INFO']   
        )  
    ])