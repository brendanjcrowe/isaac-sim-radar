"""Record ROS2 bag files for radar and LiDAR topics.

Usage:
    python3 record_bags.py [--output-dir bags/] [--duration 60]

Records /radar/point_cloud and /lidar/point_cloud topics.
"""

import argparse
import os
import subprocess
import sys
from datetime import datetime


def record(output_dir: str = "bags", duration: int = 0, topics: list = None):
    """Start ros2 bag record for the specified topics."""
    if topics is None:
        topics = ["/radar/point_cloud", "/lidar/point_cloud", "/tf", "/tf_static"]

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_name = os.path.join(output_dir, f"radar_lidar_{timestamp}")
    os.makedirs(output_dir, exist_ok=True)

    cmd = ["ros2", "bag", "record", "-o", bag_name] + topics
    if duration > 0:
        cmd.extend(["--max-duration", str(duration)])

    print(f"Recording to {bag_name}...")
    print(f"Topics: {', '.join(topics)}")
    print("Press Ctrl+C to stop.")

    try:
        subprocess.run(cmd, check=True)
    except KeyboardInterrupt:
        print(f"\nRecording saved to {bag_name}")
    except FileNotFoundError:
        print("Error: ros2 command not found. Source your ROS2 setup first.",
              file=sys.stderr)
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="Record radar/LiDAR ROS2 bags")
    parser.add_argument("--output-dir", default="bags", help="Output directory")
    parser.add_argument("--duration", type=int, default=0,
                        help="Recording duration in seconds (0=unlimited)")
    args = parser.parse_args()
    record(args.output_dir, args.duration)


if __name__ == "__main__":
    main()
