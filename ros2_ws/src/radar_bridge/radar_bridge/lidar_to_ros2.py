"""ROS2 node that bridges Isaac Sim RTX Lidar UDP output to PointCloud2.

Isaac Sim sends lidar points as LDR2 UDP packets (same struct as RDR2).

LDR2 wire format (little-endian):
  Header (16 bytes):
    char[4] magic     — b'LDR2'
    uint32  num_points — number of points in this scan
    uint64  timestamp_ns — sensor timestamp in nanoseconds
  Per-point (24 bytes each, 6 × float32):
    float32 x         — point position x (sensor frame, meters)
    float32 y         — point position y
    float32 z         — point position z
    float32 intensity — scalar intensity / reflectivity
    float32 unused1   — 0.0
    float32 unused2   — 0.0
"""

import socket
import struct

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from radar_bridge.udp_listener import create_multicast_socket, MAX_UDP_SIZE

# LDR2 wire format constants (same structure as RDR2, different magic)
LDR2_MAGIC = b"LDR2"
HEADER_FORMAT = "<4sIQ"   # magic(4s) + num_points(I) + timestamp_ns(Q)
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)   # 16 bytes
POINT_FORMAT = "<ffffff"  # x, y, z, intensity, unused, unused
POINT_SIZE = struct.calcsize(POINT_FORMAT)     # 24 bytes

LIDAR_FIELDS = [
    PointField(name="x",         offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name="y",         offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name="z",         offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
]
POINT_STEP = 16  # only 4 fields × 4 bytes for PointCloud2 (standard lidar format)


def parse_ldr2_packet(data: bytes):
    """Parse an LDR2 UDP packet. Returns (num_points, timestamp_ns, points_array) or None."""
    if len(data) < HEADER_SIZE:
        return None
    magic, num_points, timestamp_ns = struct.unpack(HEADER_FORMAT, data[:HEADER_SIZE])
    if magic != LDR2_MAGIC:
        return None
    expected = HEADER_SIZE + num_points * POINT_SIZE
    if len(data) < expected:
        num_points = (len(data) - HEADER_SIZE) // POINT_SIZE
    if num_points == 0:
        return num_points, timestamp_ns, np.empty((0, 6), dtype=np.float32)
    raw = data[HEADER_SIZE: HEADER_SIZE + num_points * POINT_SIZE]
    points = np.frombuffer(raw, dtype=np.float32).reshape(num_points, 6)
    return num_points, timestamp_ns, points


class LidarToRos2Node(Node):
    """Bridge node: lidar LDR2 UDP → ROS2 PointCloud2."""

    def __init__(self):
        super().__init__("lidar_to_ros2")

        self.declare_parameter("multicast_group", "127.0.0.1")
        self.declare_parameter("port", 10002)
        self.declare_parameter("interface", "0.0.0.0")
        self.declare_parameter("frame_id", "lidar_link")
        self.declare_parameter("topic", "/lidar/point_cloud")
        self.declare_parameter("socket_timeout", 0.05)

        self.multicast_group = self.get_parameter("multicast_group").value
        self.port = self.get_parameter("port").value
        self.interface = self.get_parameter("interface").value
        self.frame_id = self.get_parameter("frame_id").value
        topic = self.get_parameter("topic").value
        self.socket_timeout = self.get_parameter("socket_timeout").value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.publisher = self.create_publisher(PointCloud2, topic, sensor_qos)

        try:
            self.sock = create_multicast_socket(
                self.multicast_group, self.port, self.interface, self.socket_timeout
            )
            self.get_logger().info(
                f"LiDAR bridge listening on {self.multicast_group}:{self.port}"
            )
        except OSError as e:
            self.get_logger().error(f"Failed to create lidar UDP socket: {e}")
            raise

        self.timer = self.create_timer(0.01, self._poll_udp)
        self.frame_count = 0

    def _poll_udp(self):
        try:
            data, _ = self.sock.recvfrom(MAX_UDP_SIZE)
        except socket.timeout:
            return
        except OSError as e:
            self.get_logger().warn(f"Socket error: {e}")
            return

        result = parse_ldr2_packet(data)
        if result is None:
            return
        num_points, timestamp_ns, points = result

        msg = self._build_pointcloud2(num_points, timestamp_ns, points)
        self.publisher.publish(msg)

        self.frame_count += 1
        if self.frame_count % 100 == 0:
            self.get_logger().info(
                f"Published {self.frame_count} lidar scans (latest: {num_points} pts)"
            )

    def _build_pointcloud2(self, num_points, timestamp_ns, points):
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = num_points
        msg.fields = LIDAR_FIELDS
        msg.is_bigendian = False
        msg.point_step = POINT_STEP
        msg.row_step = POINT_STEP * num_points
        msg.is_dense = True

        if num_points == 0:
            msg.data = b""
            return msg

        # Pack x, y, z, intensity as float32 (standard lidar PointCloud2)
        xyzi = np.column_stack([
            points[:, 0],  # x
            points[:, 1],  # y
            points[:, 2],  # z
            points[:, 3],  # intensity
        ]).astype(np.float32)
        msg.data = xyzi.tobytes()
        return msg

    def destroy_node(self):
        if hasattr(self, "sock"):
            self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarToRos2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
