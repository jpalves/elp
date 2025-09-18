import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='elp',
            executable='camera_sub',
            name='camera_subscriber',
            output='screen'
        )
    ])
