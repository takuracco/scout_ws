from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cmd_vel_joy',          
            executable='joy_cmd',
            name='joy_cmd',  
            output='screen'    
        ),
        Node(
            package='joy',          
            executable='joy_node',
            name='joy_node',  
            output='screen'    
        )
    ])