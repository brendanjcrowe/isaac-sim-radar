"""Tests for the UDP listener and RDR2 packet parser."""

import struct
import socket
import threading
import time
import unittest

import numpy as np

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "radar_bridge"))

from radar_bridge.udp_listener import (
    HEADER_FORMAT,
    HEADER_MAGIC,
    HEADER_SIZE,
    DETECTION_FORMAT,
    DETECTION_SIZE,
    RadarFrame,
    parse_generic_model_output,
    create_multicast_socket,
)


def build_packet(timestamp_ns=1000000, num_detections=5):
    """Build a mock RDR2 binary packet."""
    header = struct.pack(HEADER_FORMAT, HEADER_MAGIC, num_detections, timestamp_ns)

    detections = b""
    for i in range(num_detections):
        det = struct.pack(
            DETECTION_FORMAT,
            float(i),           # x
            float(i * 0.5),     # y
            0.4,                # z
            1.5 + i * 0.1,      # velocity
            -10.0 + i,          # rcs
            15.0 + i * 2,       # snr
        )
        detections += det

    return header + detections


class TestPacketParsing(unittest.TestCase):

    def test_header_size(self):
        self.assertEqual(HEADER_SIZE, 16)

    def test_detection_size(self):
        self.assertEqual(DETECTION_SIZE, 24)

    def test_parse_valid_packet(self):
        packet = build_packet(timestamp_ns=999, num_detections=3)
        frame = parse_generic_model_output(packet)

        self.assertIsNotNone(frame)
        self.assertEqual(frame.timestamp_ns, 999)
        self.assertEqual(frame.num_detections, 3)

    def test_detection_values(self):
        packet = build_packet(num_detections=4)
        frame = parse_generic_model_output(packet)

        self.assertEqual(len(frame.x), 4)
        np.testing.assert_allclose(frame.x, [0.0, 1.0, 2.0, 3.0])
        np.testing.assert_allclose(frame.y, [0.0, 0.5, 1.0, 1.5])
        np.testing.assert_allclose(frame.z, [0.4, 0.4, 0.4, 0.4])
        np.testing.assert_allclose(frame.velocity, [1.5, 1.6, 1.7, 1.8])
        np.testing.assert_allclose(frame.rcs, [-10.0, -9.0, -8.0, -7.0])
        np.testing.assert_allclose(frame.snr, [15.0, 17.0, 19.0, 21.0])

    def test_zero_detections(self):
        packet = build_packet(num_detections=0)
        frame = parse_generic_model_output(packet)

        self.assertIsNotNone(frame)
        self.assertEqual(frame.num_detections, 0)
        self.assertEqual(len(frame.x), 0)

    def test_truncated_header(self):
        frame = parse_generic_model_output(b"\x00" * 10)
        self.assertIsNone(frame)

    def test_bad_magic_returns_none(self):
        """Unknown magic → returns None (no fallback parser)."""
        bad_header = struct.pack("<4sIQ", b"XXXX", 1, 0)
        det = struct.pack(DETECTION_FORMAT, *([1.0] * 6))
        frame = parse_generic_model_output(bad_header + det)
        self.assertIsNone(frame)

    def test_truncated_detections(self):
        """Packet claims 10 detections but only has data for 3."""
        header = struct.pack(HEADER_FORMAT, HEADER_MAGIC, 10, 0)
        det_data = b"\x00" * (DETECTION_SIZE * 3)
        packet = header + det_data

        frame = parse_generic_model_output(packet)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.num_detections, 3)

    def test_large_frame(self):
        packet = build_packet(num_detections=500)
        frame = parse_generic_model_output(packet)

        self.assertIsNotNone(frame)
        self.assertEqual(frame.num_detections, 500)
        self.assertEqual(len(frame.x), 500)


class TestUDPSocket(unittest.TestCase):
    """Test actual UDP send/receive with mock data on localhost."""

    def test_send_receive_localhost(self):
        """Send a mock packet via UDP and receive it."""
        port = 19999  # Use a high port to avoid conflicts

        recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        recv_sock.bind(("127.0.0.1", port))
        recv_sock.settimeout(2.0)

        packet = build_packet(num_detections=10)

        def send():
            time.sleep(0.1)
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.sendto(packet, ("127.0.0.1", port))
            s.close()

        t = threading.Thread(target=send)
        t.start()

        data, addr = recv_sock.recvfrom(65535)
        recv_sock.close()
        t.join()

        frame = parse_generic_model_output(data)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.num_detections, 10)


if __name__ == "__main__":
    unittest.main()
