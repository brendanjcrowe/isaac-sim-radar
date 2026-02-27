"""Nav2 autonomous navigation launch.

Starts the full Nav2 stack for waypoint-following in the urban scene:
  - controller_server  (DWB local planner)
  - planner_server     (NavFn global planner)
  - recoveries_server  (spin / backup / wait)
  - bt_navigator       (behaviour-tree-driven goal execution)
  - lifecycle_manager  (coordinates all Nav2 node state transitions)

Prerequisites (must already be running):
  - Isaac Sim with the scene loaded and playing
  - SLAM Toolbox producing a /map and odom → base_link TF
    (launch via: ros2 launch /app/launch/slam.launch.py)
  - /odom topic from the robot

Usage:
    # Full pipeline: start SLAM first, then Nav2
    ros2 launch /app/launch/slam.launch.py &
    ros2 launch /app/launch/nav2.launch.py

    # To navigate to a pose from the CLI:
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
      "pose: {header: {frame_id: map}, pose: {position: {x: 20.0, y: 5.0}}}"
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    nav2_params_file = os.path.join(project_dir, "config", "nav2_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file  = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock (/clock) published by Isaac Sim.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=nav2_params_file,
            description="Full path to the Nav2 params YAML file.",
        ),

        # ── Controller Server ─────────────────────────────────────────────────
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            remappings=[("cmd_vel", "/cmd_vel")],
        ),

        # ── Planner Server ────────────────────────────────────────────────────
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),

        # ── Recoveries Server ─────────────────────────────────────────────────
        Node(
            package="nav2_recoveries",
            executable="recoveries_server",
            name="recoveries_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),

        # ── BT Navigator ──────────────────────────────────────────────────────
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),

        # ── Lifecycle Manager — manages state of all Nav2 nodes ───────────────
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": [
                    "planner_server",
                    "controller_server",
                    "recoveries_server",
                    "bt_navigator",
                ],
            }],
        ),
    ])
