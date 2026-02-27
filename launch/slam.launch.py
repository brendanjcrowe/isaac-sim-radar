"""Standalone SLAM launch — LiDAR mapping without Nav2 autonomous navigation.

Starts:
  1. pointcloud_to_laserscan  — converts /lidar/point_cloud (PointCloud2) → /scan (LaserScan)
  2. slam_toolbox online_sync — builds a 2D occupancy map from /scan

Isaac Sim must be running and producing /lidar/point_cloud before this launch.
The odom → base_link TF must also be present (published by the robot's odometry or
the static publisher in full_stack.launch.py during development).

Usage:
    ros2 launch /app/launch/slam.launch.py
    ros2 launch /app/launch/slam.launch.py use_sim_time:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    slam_params_file = os.path.join(project_dir, "config", "slam_toolbox.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock (/clock) published by Isaac Sim.",
        ),

        # ── PointCloud2 → LaserScan ──────────────────────────────────────────
        # The OS1-64 LiDAR produces a full 3D point cloud. SLAM Toolbox works
        # on 2D LaserScans. pointcloud_to_laserscan takes a horizontal slice
        # (min_height..max_height) and projects it into a LaserScan.
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            remappings=[
                ("cloud_in", "/lidar/point_cloud"),
                ("scan",     "/scan"),
            ],
            parameters=[{
                "use_sim_time":   use_sim_time,
                "target_frame":   "lidar_link",
                "transform_tolerance": 0.01,
                # Height band to extract for the 2D slice (meters, in lidar_link frame)
                "min_height": -0.2,
                "max_height":  0.5,
                # Angular resolution of the output LaserScan
                "angle_min":   -3.14159,   # -180 deg
                "angle_max":    3.14159,   # +180 deg
                "angle_increment": 0.00872,  # ~0.5 deg; 720 rays/scan
                # Range limits
                "scan_time":  0.1,          # 10 Hz to match LiDAR publish rate
                "range_min":  0.45,
                "range_max":  50.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
            }],
            output="screen",
        ),

        # ── SLAM Toolbox ──────────────────────────────────────────────────────
        Node(
            package="slam_toolbox",
            executable="sync_slam_toolbox_node",
            name="slam_toolbox",
            parameters=[slam_params_file, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
    ])
