"""Integration tests: full UDP-packet → parsed-frame → packed-bytes pipeline.

No ROS2 runtime, no Isaac Sim, no mocking required — pure stdlib + numpy.
These tests verify the complete data path that the radar_to_ros2 node runs
on every received UDP packet, catching layout or dtype regressions before
containers are even built.
"""

import struct
import sys
import unittest

import numpy as np

# Allow running from project root without installing the package
sys.path.insert(0, "ros2_ws/src/radar_bridge")

from radar_bridge.udp_listener import (
    HEADER_FORMAT,
    HEADER_MAGIC,
    HEADER_SIZE,
    DETECTION_SIZE,
    parse_generic_model_output,
)

# Replicate the node's field ordering and point step (from radar_to_ros2.py)
# [x, y, z, velocity, rcs, snr] — 6 float32 = 24 bytes per point
_FIELD_ORDER = ["x", "y", "z", "velocity", "rcs", "snr"]
_POINT_STEP = 24  # 6 * 4


def _make_packet(detections: list[dict]) -> bytes:
    """Build a valid GenericModelOutput binary packet from a list of detection dicts.

    Each dict must have keys: x, y, z, velocity, range, azimuth, elevation, rcs, snr.
    """
    num = len(detections)
    # Header: magic, version, sensor_id, timestamp_ns, sx, sy, sz, num_detections, reserved
    header = struct.pack(HEADER_FORMAT, HEADER_MAGIC, 1, 42, 123456789, 0.0, 0.0, 0.0, num, 0)
    body = b""
    for d in detections:
        body += struct.pack(
            "<ffffffffffff",
            d["x"], d["y"], d["z"], d["velocity"],
            d["range"], d["azimuth"], d["elevation"],
            d["rcs"], d["snr"],
            0.0, 0.0, 0.0,  # noise_floor, reserved1, reserved2
        )
    return header + body


def _pack_frame(frame) -> bytes:
    """Replicate radar_to_ros2.RadarToRos2Node._frame_to_pointcloud2 data packing."""
    if frame.num_detections == 0:
        return b""
    points = np.column_stack([
        frame.x, frame.y, frame.z,
        frame.velocity, frame.rcs, frame.snr,
    ]).astype(np.float32)
    return points.tobytes()


class TestFullBytePipeline(unittest.TestCase):
    """Roundtrip: binary packet → RadarFrame → packed PointCloud2 bytes."""

    def setUp(self):
        self.detections = [
            {"x": 1.0, "y": 2.0, "z": 0.5, "velocity": -3.5,
             "range": 5.0, "azimuth": 0.1, "elevation": 0.05, "rcs": -10.0, "snr": 15.0},
            {"x": -4.0, "y": 0.0, "z": 1.2, "velocity": 0.0,
             "range": 8.0, "azimuth": -0.2, "elevation": 0.0, "rcs": -5.0, "snr": 20.0},
            {"x": 0.0, "y": 7.5, "z": 0.0, "velocity": 2.1,
             "range": 12.0, "azimuth": 0.4, "elevation": -0.1, "rcs": 3.0, "snr": 8.0},
        ]
        self.packet = _make_packet(self.detections)

    def test_parse_recovers_xyz(self):
        frame = parse_generic_model_output(self.packet)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.num_detections, 3)
        np.testing.assert_allclose(frame.x, [1.0, -4.0, 0.0], rtol=1e-6)
        np.testing.assert_allclose(frame.y, [2.0, 0.0, 7.5], rtol=1e-6)
        np.testing.assert_allclose(frame.z, [0.5, 1.2, 0.0], rtol=1e-6)

    def test_parse_recovers_rcs_snr(self):
        frame = parse_generic_model_output(self.packet)
        np.testing.assert_allclose(frame.rcs, [-10.0, -5.0, 3.0], rtol=1e-6)
        np.testing.assert_allclose(frame.snr, [15.0, 20.0, 8.0], rtol=1e-6)

    def test_packed_bytes_roundtrip(self):
        """Parse → pack → unpack recovers all six fields for every detection."""
        frame = parse_generic_model_output(self.packet)
        packed = _pack_frame(frame)

        n = frame.num_detections
        unpacked = np.frombuffer(packed, dtype=np.float32).reshape(n, 6)

        for i, d in enumerate(self.detections):
            self.assertAlmostEqual(unpacked[i, 0], d["x"], places=5, msg="x mismatch")
            self.assertAlmostEqual(unpacked[i, 1], d["y"], places=5, msg="y mismatch")
            self.assertAlmostEqual(unpacked[i, 2], d["z"], places=5, msg="z mismatch")
            self.assertAlmostEqual(unpacked[i, 3], d["velocity"], places=5, msg="velocity mismatch")
            self.assertAlmostEqual(unpacked[i, 4], d["rcs"], places=5, msg="rcs mismatch")
            self.assertAlmostEqual(unpacked[i, 5], d["snr"], places=5, msg="snr mismatch")


class TestPointCloud2DataLayout(unittest.TestCase):
    """Verify the packed byte layout matches the PointCloud2 field spec."""

    def setUp(self):
        det = {"x": 10.0, "y": 20.0, "z": 30.0, "velocity": 1.5,
               "range": 5.0, "azimuth": 0.0, "elevation": 0.0, "rcs": -8.0, "snr": 12.0}
        packet = _make_packet([det])
        self.frame = parse_generic_model_output(packet)
        self.packed = _pack_frame(self.frame)

    def test_byte_length_equals_point_step(self):
        n = self.frame.num_detections
        self.assertEqual(len(self.packed), n * _POINT_STEP)

    def test_field_offsets(self):
        """Each field sits at the documented offset within the packed bytes."""
        expected = {"x": 10.0, "y": 20.0, "z": 30.0, "velocity": 1.5, "rcs": -8.0, "snr": 12.0}
        offsets = {"x": 0, "y": 4, "z": 8, "velocity": 12, "rcs": 16, "snr": 20}
        for name, offset in offsets.items():
            (value,) = struct.unpack_from("<f", self.packed, offset)
            self.assertAlmostEqual(value, expected[name], places=5,
                                   msg=f"Field '{name}' wrong at offset {offset}")

    def test_all_fields_are_float32(self):
        """All 6 fields per point must be 4-byte floats (PointField.FLOAT32 = 7)."""
        n = self.frame.num_detections
        arr = np.frombuffer(self.packed, dtype=np.float32)
        self.assertEqual(arr.dtype, np.float32)
        self.assertEqual(len(arr), n * 6)


class TestZeroDetections(unittest.TestCase):
    """Edge case: packet with zero detections produces empty packed bytes."""

    def test_zero_detections_empty_bytes(self):
        packet = _make_packet([])
        frame = parse_generic_model_output(packet)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.num_detections, 0)
        packed = _pack_frame(frame)
        self.assertEqual(packed, b"")

    def test_zero_detections_empty_arrays(self):
        packet = _make_packet([])
        frame = parse_generic_model_output(packet)
        self.assertEqual(len(frame.x), 0)
        self.assertEqual(len(frame.rcs), 0)


if __name__ == "__main__":
    unittest.main()
