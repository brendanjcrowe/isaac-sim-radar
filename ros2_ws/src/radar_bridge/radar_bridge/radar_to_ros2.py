"""ROS2 node that bridges Isaac Sim RTX Radar UDP output to PointCloud2.

Listens on a UDP multicast socket for GenericModelOutput packets from the
TranscoderRadar OmniGraph node, parses detections, and publishes them as
sensor_msgs/PointCloud2 messages with custom fields (x, y, z, velocity,
rcs, snr).
"""

import socket
import struct

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from radar_bridge.udp_listener import (
    MAX_UDP_SIZE,
    RadarFrame,
    create_multicast_socket,
    parse_generic_model_output,
)


# PointCloud2 field definitions for radar detections
RADAR_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="velocity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="rcs", offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name="snr", offset=20, datatype=PointField.FLOAT32, count=1),
]
POINT_STEP = 24  # 6 floats * 4 bytes


class RadarToRos2Node(Node):
    """Bridge node: radar UDP → ROS2 PointCloud2."""

    def __init__(self):
        super().__init__("radar_to_ros2")

        # Declare parameters
        self.declare_parameter("multicast_group", "239.0.0.1")
        self.declare_parameter("port", 10001)
        self.declare_parameter("interface", "0.0.0.0")
        self.declare_parameter("frame_id", "radar_link")
        self.declare_parameter("topic", "/radar/point_cloud")
        self.declare_parameter("socket_timeout", 0.05)

        # Read parameters
        self.multicast_group = self.get_parameter("multicast_group").value
        self.port = self.get_parameter("port").value
        self.interface = self.get_parameter("interface").value
        self.frame_id = self.get_parameter("frame_id").value
        topic = self.get_parameter("topic").value
        self.socket_timeout = self.get_parameter("socket_timeout").value

        # Publisher with sensor data QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.publisher = self.create_publisher(PointCloud2, topic, sensor_qos)

        # Open UDP socket
        try:
            self.sock = create_multicast_socket(
                self.multicast_group,
                self.port,
                self.interface,
                self.socket_timeout,
            )
            self.get_logger().info(
                f"Listening on {self.multicast_group}:{self.port} "
                f"(interface {self.interface})"
            )
        except OSError as e:
            self.get_logger().error(f"Failed to create UDP socket: {e}")
            raise

        # Timer to poll the socket
        self.timer = self.create_timer(0.01, self._poll_udp)  # 100 Hz polling

        self.frame_count = 0

    def _poll_udp(self):
        """Poll the UDP socket and publish any received frames."""
        try:
            data, addr = self.sock.recvfrom(MAX_UDP_SIZE)
        except socket.timeout:
            return
        except OSError as e:
            self.get_logger().warn(f"Socket error: {e}")
            return

        frame = parse_generic_model_output(data)
        if frame is None:
            return

        msg = self._frame_to_pointcloud2(frame)
        self.publisher.publish(msg)

        self.frame_count += 1
        if self.frame_count % 100 == 0:
            self.get_logger().info(
                f"Published {self.frame_count} frames "
                f"(latest: {frame.num_detections} detections)"
            )

    def _frame_to_pointcloud2(self, frame: RadarFrame) -> PointCloud2:
        """Convert a RadarFrame to a PointCloud2 message."""
        msg = PointCloud2()

        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        n = frame.num_detections
        msg.height = 1
        msg.width = n
        msg.fields = RADAR_FIELDS
        msg.is_bigendian = False
        msg.point_step = POINT_STEP
        msg.row_step = POINT_STEP * n
        msg.is_dense = True

        if n == 0:
            msg.data = b""
            return msg

        # Pack point data: [x, y, z, velocity, rcs, snr] per point
        points = np.column_stack([
            frame.x,
            frame.y,
            frame.z,
            frame.velocity,
            frame.rcs,
            frame.snr,
        ]).astype(np.float32)

        msg.data = points.tobytes()
        return msg

    def destroy_node(self):
        """Clean up the UDP socket."""
        if hasattr(self, "sock"):
            self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RadarToRos2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
