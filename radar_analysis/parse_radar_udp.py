"""Standalone radar UDP parser for offline analysis and testing.

Can run independently of ROS2 to capture and inspect radar data:
    python3 parse_radar_udp.py [--port 10001] [--duration 10]
"""

import argparse
import os
import sys
import time

# Add parent so we can import from the radar_bridge package (works from any CWD)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ros2_ws", "src", "radar_bridge"))

from radar_bridge.udp_listener import (
    create_multicast_socket,
    parse_generic_model_output,
    MAX_UDP_SIZE,
)


def main():
    parser = argparse.ArgumentParser(description="Capture and display radar UDP data")
    parser.add_argument("--group", default="239.0.0.1", help="Multicast group")
    parser.add_argument("--port", type=int, default=10001, help="UDP port")
    parser.add_argument("--duration", type=float, default=10.0, help="Capture duration (seconds)")
    args = parser.parse_args()

    print(f"Listening on {args.group}:{args.port} for {args.duration}s...")

    sock = create_multicast_socket(args.group, args.port, timeout=0.5)
    start = time.time()
    frame_count = 0
    total_detections = 0

    try:
        while time.time() - start < args.duration:
            try:
                data, addr = sock.recvfrom(MAX_UDP_SIZE)
            except TimeoutError:
                continue

            frame = parse_generic_model_output(data)
            if frame is None:
                continue

            frame_count += 1
            total_detections += frame.num_detections

            print(
                f"Frame {frame_count}: {frame.num_detections} detections, "
                f"sensor_id={frame.sensor_id}, "
                f"timestamp={frame.timestamp_ns}"
            )

            if frame.num_detections > 0:
                print(f"  Range: [{frame.range_m.min():.2f}, {frame.range_m.max():.2f}] m")
                print(f"  RCS:   [{frame.rcs.min():.1f}, {frame.rcs.max():.1f}] dBsm")
                print(f"  SNR:   [{frame.snr.min():.1f}, {frame.snr.max():.1f}] dB")

    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        elapsed = time.time() - start
        print(f"\nCaptured {frame_count} frames ({total_detections} total detections) "
              f"in {elapsed:.1f}s ({frame_count/max(elapsed, 0.001):.1f} fps)")


if __name__ == "__main__":
    main()
