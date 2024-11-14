import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    this_dir = get_package_share_directory('riptide_slam')

    return LaunchDescription([
        Node(
            package='riptide_slam',
            executable='riptide_slam',
            output='screen',
            parameters=[
            ]
        )
    ])