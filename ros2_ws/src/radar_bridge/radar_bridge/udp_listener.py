"""UDP listener for Isaac Sim RTX Radar GenericModelOutput packets.

The TranscoderRadar OmniGraph node sends radar detections as UDP packets
in the GenericModelOutput binary format. This module handles socket setup
and binary parsing.

GenericModelOutput wire format (little-endian):
  Header (40 bytes):
    uint32  magic          — 0x52415444 ('RATD')
    uint32  version        — format version
    uint32  sensor_id      — sensor identifier
    uint64  timestamp_ns   — simulation timestamp in nanoseconds
    float32 sensor_x       — sensor world position x
    float32 sensor_y       — sensor world position y
    float32 sensor_z       — sensor world position z
    uint32  num_detections — number of detections in this frame
    uint32  reserved       — padding
  Per-detection (48 bytes each):
    float32 x              — detection position x (sensor frame, meters)
    float32 y              — detection position y
    float32 z              — detection position z
    float32 velocity       — radial velocity (m/s, positive = approaching)
    float32 range          — range to detection (meters)
    float32 azimuth        — azimuth angle (radians)
    float32 elevation      — elevation angle (radians)
    float32 rcs            — radar cross section (dBsm)
    float32 snr            — signal-to-noise ratio (dB)
    float32 noise_floor    — noise floor (dBm)
    float32 reserved1
    float32 reserved2

Note: The exact binary layout depends on the Isaac Sim version and radar
extension build. If detections don't parse correctly, adjust HEADER_SIZE,
DETECTION_SIZE, and struct format strings to match your installation.
"""

import socket
import struct
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

# Binary format constants
HEADER_MAGIC = 0x52415444  # 'RATD'
HEADER_FORMAT = "<IIIQfffII"  # little-endian
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)  # 40 bytes

DETECTION_FORMAT = "<ffffffffffff"  # 12 floats
DETECTION_SIZE = struct.calcsize(DETECTION_FORMAT)  # 48 bytes

# Field indices in detection tuple
DET_X = 0
DET_Y = 1
DET_Z = 2
DET_VELOCITY = 3
DET_RANGE = 4
DET_AZIMUTH = 5
DET_ELEVATION = 6
DET_RCS = 7
DET_SNR = 8

MAX_UDP_SIZE = 65535


@dataclass
class RadarFrame:
    """Parsed radar frame from a GenericModelOutput UDP packet."""

    sensor_id: int = 0
    timestamp_ns: int = 0
    sensor_position: tuple = (0.0, 0.0, 0.0)
    num_detections: int = 0
    # Arrays: each (N,) shaped
    x: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.float32))
    y: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.float32))
    z: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.float32))
    velocity: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.float32))
    range_m: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.float32))
    azimuth: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.float32))
    elevation: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.float32))
    rcs: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.float32))
    snr: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.float32))


def parse_generic_model_output(data: bytes) -> Optional[RadarFrame]:
    """Parse a GenericModelOutput binary packet into a RadarFrame.

    Returns None if the packet is malformed or has an unrecognized magic number.
    """
    if len(data) < HEADER_SIZE:
        return None

    header = struct.unpack(HEADER_FORMAT, data[:HEADER_SIZE])
    magic = header[0]

    # If magic doesn't match, try parsing without magic check (some versions
    # may use a different magic or no magic at all)
    if magic != HEADER_MAGIC:
        # Fall back: treat the entire payload as flat detection data
        return _parse_flat_detections(data)

    sensor_id = header[2]
    timestamp_ns = header[3]
    sensor_x, sensor_y, sensor_z = header[4], header[5], header[6]
    num_detections = header[7]

    expected_size = HEADER_SIZE + num_detections * DETECTION_SIZE
    if len(data) < expected_size:
        # Truncated packet — parse what we can
        num_detections = (len(data) - HEADER_SIZE) // DETECTION_SIZE

    frame = RadarFrame(
        sensor_id=sensor_id,
        timestamp_ns=timestamp_ns,
        sensor_position=(sensor_x, sensor_y, sensor_z),
        num_detections=num_detections,
    )

    if num_detections == 0:
        return frame

    # Parse all detections at once using numpy for speed
    det_data = data[HEADER_SIZE : HEADER_SIZE + num_detections * DETECTION_SIZE]
    detections = np.frombuffer(det_data, dtype=np.float32).reshape(num_detections, 12)

    frame.x = detections[:, DET_X]
    frame.y = detections[:, DET_Y]
    frame.z = detections[:, DET_Z]
    frame.velocity = detections[:, DET_VELOCITY]
    frame.range_m = detections[:, DET_RANGE]
    frame.azimuth = detections[:, DET_AZIMUTH]
    frame.elevation = detections[:, DET_ELEVATION]
    frame.rcs = detections[:, DET_RCS]
    frame.snr = detections[:, DET_SNR]

    return frame


def _parse_flat_detections(data: bytes) -> Optional[RadarFrame]:
    """Fallback parser: treat entire payload as an array of detection structs."""
    num_detections = len(data) // DETECTION_SIZE
    if num_detections == 0:
        return None

    frame = RadarFrame(num_detections=num_detections)
    det_data = data[: num_detections * DETECTION_SIZE]
    detections = np.frombuffer(det_data, dtype=np.float32).reshape(num_detections, 12)

    frame.x = detections[:, DET_X]
    frame.y = detections[:, DET_Y]
    frame.z = detections[:, DET_Z]
    frame.velocity = detections[:, DET_VELOCITY]
    frame.range_m = detections[:, DET_RANGE]
    frame.azimuth = detections[:, DET_AZIMUTH]
    frame.elevation = detections[:, DET_ELEVATION]
    frame.rcs = detections[:, DET_RCS]
    frame.snr = detections[:, DET_SNR]

    return frame


def create_multicast_socket(
    multicast_group: str = "239.0.0.1",
    port: int = 10001,
    interface: str = "0.0.0.0",
    timeout: float = 0.1,
) -> socket.socket:
    """Create and bind a UDP multicast socket for receiving radar data."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", port))

    # Join multicast group
    mreq = struct.pack(
        "4s4s",
        socket.inet_aton(multicast_group),
        socket.inet_aton(interface),
    )
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    sock.settimeout(timeout)

    return sock
