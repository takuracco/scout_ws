from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    scout_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('scout_mini_ROS2'),
                'launch',
                'launch.py'
            )
        )
    )

    cost_node = Node(
        package='cost_cul',
        executable='cost_cul_node',
        output='screen'
    )

    return LaunchDescription([
        scout_launch,
        cost_node
    ])