import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='elp',
            executable='camera_pub',
            name='camera_publisher',
            prefix="gdbserver localhost:3000",
            parameters=[os.path.join(
                get_package_share_directory('elp'),
                'config',
                'camera_params.yaml'
            )]
        )
    ])
