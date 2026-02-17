"""Master launch file: radar bridge + static TF publishers + RViz2.

Usage:
    ros2 launch full_stack.launch.py

Note: Isaac Sim must be started separately (it's not a ROS2 node).
Start Isaac Sim first, load the scene (launch_scene.py), press Play,
then run this launch file.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    rviz_config = os.path.join(project_dir, "rviz", "radar_comparison.rviz")

    # Radar bridge launch
    radar_bridge_share = get_package_share_directory("radar_bridge")
    radar_bridge_launch = os.path.join(
        radar_bridge_share, "launch", "radar_bridge.launch.py"
    )

    return LaunchDescription([
        # --- Radar bridge ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(radar_bridge_launch),
        ),

        # --- Static TF: map → odom ---
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        ),

        # --- Static TF: base_link → radar_link ---
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_radar",
            arguments=["0.3", "0", "0.4", "0", "0", "0", "base_link", "radar_link"],
        ),

        # --- Static TF: base_link → lidar_link ---
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_lidar",
            arguments=["0", "0", "0.5", "0", "0", "0", "base_link", "lidar_link"],
        ),

        # --- RViz2 ---
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
