"""UDP listener for Isaac Sim RTX Radar packets.

Isaac Sim sends radar detections as UDP packets in the RDR2 binary format.
The RDR2 packet is produced by run_headless.py using NVIDIA's get_gmo_data()
parser (isaacsim.sensors.rtx) to extract detections from the raw OMGN
GenericModelOutput buffer, then packing them into this compact wire format.

RDR2 wire format (little-endian):
  Header (16 bytes):
    char[4] magic          — b'RDR2'
    uint32  num_detections — number of detections in this frame
    uint64  timestamp_ns   — sensor timestamp in nanoseconds
  Per-detection (24 bytes each, 6 × float32):
    float32 x              — detection position x (sensor frame, meters)
    float32 y              — detection position y
    float32 z              — detection position z
    float32 velocity       — radial velocity (m/s)
    float32 rcs            — radar cross section / scalar intensity (dBsm)
    float32 snr            — signal-to-noise ratio (dB, 0.0 if unavailable)
"""

import socket
import struct
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

# Binary format constants
HEADER_MAGIC = b"RDR2"
HEADER_FORMAT = "<4sIQ"   # magic(4s) + num_detections(I) + timestamp_ns(Q)
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)  # 16 bytes

DETECTION_FORMAT = "<ffffff"  # x, y, z, velocity, rcs, snr
DETECTION_SIZE = struct.calcsize(DETECTION_FORMAT)  # 24 bytes

# Field indices within each detection tuple
DET_X = 0
DET_Y = 1
DET_Z = 2
DET_VELOCITY = 3
DET_RCS = 4
DET_SNR = 5

MAX_UDP_SIZE = 65535


@dataclass
class RadarFrame:
    """Parsed radar frame from an RDR2 UDP packet."""

    timestamp_ns: int = 0
    num_detections: int = 0
    # Per-detection arrays, each shape (N,) float32
    x: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.float32))
    y: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.float32))
    z: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.float32))
    velocity: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.float32))
    rcs: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.float32))
    snr: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.float32))


def parse_generic_model_output(data: bytes) -> Optional[RadarFrame]:
    """Parse an RDR2 binary packet into a RadarFrame.

    Returns None if the packet is malformed or the magic doesn't match.
    """
    if len(data) < HEADER_SIZE:
        return None

    magic, num_detections, timestamp_ns = struct.unpack(HEADER_FORMAT, data[:HEADER_SIZE])
    if magic != HEADER_MAGIC:
        return None

    # Clamp to however many complete detections actually fit in the payload
    max_det = (len(data) - HEADER_SIZE) // DETECTION_SIZE
    if num_detections > max_det:
        num_detections = max_det

    frame = RadarFrame(
        timestamp_ns=timestamp_ns,
        num_detections=num_detections,
    )

    if num_detections == 0:
        return frame

    det_bytes = data[HEADER_SIZE: HEADER_SIZE + num_detections * DETECTION_SIZE]
    detections = np.frombuffer(det_bytes, dtype=np.float32).reshape(num_detections, 6)

    frame.x = detections[:, DET_X]
    frame.y = detections[:, DET_Y]
    frame.z = detections[:, DET_Z]
    frame.velocity = detections[:, DET_VELOCITY]
    frame.rcs = detections[:, DET_RCS]
    frame.snr = detections[:, DET_SNR]

    return frame


def create_multicast_socket(
    multicast_group: str = "127.0.0.1",
    port: int = 10001,
    interface: str = "0.0.0.0",
    timeout: float = 0.1,
) -> socket.socket:
    """Create and bind a UDP socket for receiving radar/lidar data.

    Supports both multicast (239.x.x.x) and unicast (127.0.0.1) addresses.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", port))

    # Only join multicast group if address is actually multicast (224.0.0.0/4)
    first_octet = int(multicast_group.split(".")[0])
    if 224 <= first_octet <= 239:
        mreq = struct.pack(
            "4s4s",
            socket.inet_aton(multicast_group),
            socket.inet_aton(interface),
        )
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    sock.settimeout(timeout)

    return sock
