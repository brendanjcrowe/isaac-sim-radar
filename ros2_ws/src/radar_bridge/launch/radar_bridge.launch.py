"""Launch file for the radar bridge node."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("radar_bridge")
    default_config = os.path.join(pkg_share, "config", "radar_bridge.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Path to radar bridge config YAML",
        ),
        Node(
            package="radar_bridge",
            executable="radar_to_ros2",
            name="radar_to_ros2",
            parameters=[LaunchConfiguration("config_file")],
            output="screen",
        ),
    ])
