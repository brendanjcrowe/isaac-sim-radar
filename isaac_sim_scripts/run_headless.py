"""Headless Isaac Sim simulation runner.

SimulationApp MUST be instantiated before any omni.* imports — that is the
"SimulationApp-first" constraint enforced by NVIDIA's kit framework.

This script:
  1. Parses CLI args (no omni deps required).
  2. Creates SimulationApp in headless mode.
  3. Enables required extensions.
  4. Builds urban scene + attaches radar/lidar sensors.
  5. Plays the timeline and runs the update loop.
  6. Exits cleanly on SIGINT/SIGTERM or when --duration expires.

Usage (inside isaac-sim container):
    /isaac-sim/python.sh /app/isaac_sim_scripts/run_headless.py
    /isaac-sim/python.sh /app/isaac_sim_scripts/run_headless.py --duration 60
    /isaac-sim/python.sh /app/isaac_sim_scripts/run_headless.py --no-ros2
"""

import argparse
import os
import signal
import socket
import struct
import sys
import time

# Ensure our sibling scripts (enable_extensions, launch_scene) are importable
# regardless of which directory python.sh is invoked from.
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

# ── Parse CLI args before creating SimulationApp (no omni deps needed) ─────
parser = argparse.ArgumentParser(
    description="Run Isaac Sim headless with radar + lidar urban scene."
)
parser.add_argument(
    "--duration",
    type=float,
    default=0.0,
    metavar="SECONDS",
    help="Exit after this many seconds (0 = run until SIGINT/SIGTERM).",
)
parser.add_argument(
    "--no-ros2",
    action="store_true",
    help="Skip LiDAR ROS2 publisher. Use when testing without the ros2-bridge container.",
)
parser.add_argument(
    "--weather",
    choices=["clear", "fog", "rain"],
    default="clear",
    help="Weather preset (default: clear). fog degrades LiDAR; rain adds radar clutter.",
)
parser.add_argument(
    "--fog-preset",
    choices=["light", "dense"],
    default="light",
    help="Fog density preset, used only when --weather=fog (default: light).",
)
parser.add_argument(
    "--dynamic-objects",
    action="store_true",
    help="Add animated pedestrians and vehicles with baked USD time-sample animation.",
)
args = parser.parse_args()

# ── SimulationApp MUST be created before all omni.* imports ─────────────────
from isaacsim import SimulationApp  # noqa: E402  (intentional late import)

# WpmDmatApprox radar requires MotionBVH (ray-tracing BVH for moving objects).
# Pass this flag at startup so the RTX renderer initialises MotionBVH structures
# before the OmniRadar plugin loads — without it the plugin crashes.
simulation_app = SimulationApp({
    "headless": True,
    "anti_aliasing": 0,
    "extra_args": ["--/renderer/raytracingMotion/enabled=true"],
})

# ── Safe to import omni.* now ───────────────────────────────────────────────
import carb  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import UsdGeom  # noqa: E402

import numpy as np  # noqa: E402

from enable_extensions import enable_extensions  # noqa: E402
from launch_scene import (  # noqa: E402
    add_dynamic_objects,
    add_weather_effects,
    attach_lidar_sensor,
    attach_radar_sensor,
    create_ground_plane,
    create_urban_environment,
    load_config,
    setup_lidar_ros2_publisher as setup_lidar_annotator,
    setup_radar_annotator,
    spawn_robot,
)

# ── SIGINT / SIGTERM handler ─────────────────────────────────────────────────
_shutdown = False


def _handle_signal(signum, frame):
    global _shutdown
    carb.log_warn(f"Signal {signum} received — will exit after current frame")
    _shutdown = True


signal.signal(signal.SIGINT, _handle_signal)
signal.signal(signal.SIGTERM, _handle_signal)

# ── Enable extensions ────────────────────────────────────────────────────────
carb.log_info("=== Isaac Sim Radar Runner: enabling extensions ===")
enable_extensions()

# get_gmo_data is provided by the isaacsim.sensors.rtx extension (loaded above).
# It parses the raw OMGN bytes returned by the GenericModelOutput annotator into
# a structured object with fields: x, y, z, scalar, velocities, numElements, etc.
try:
    from isaacsim.sensors.rtx import get_gmo_data as _get_gmo_data
    _GMO_MAGIC = 0x4E474D4F  # 'OMGN' as little-endian uint32
    carb.log_info("get_gmo_data available — will send structured RDR2 packets")
except ImportError:
    _get_gmo_data = None
    _GMO_MAGIC = None
    carb.log_warn("get_gmo_data not available — falling back to raw GMO bytes")

# ── Build scene ──────────────────────────────────────────────────────────────
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

carb.log_info("=== Building urban scene ===")
create_ground_plane(stage)
create_urban_environment(stage)
robot_path = spawn_robot(stage)

# Create lidar FIRST: IsaacSensorCreateRtxLidar may block for ~140s waiting
# for the NVIDIA CDN (OS1 USD asset).  Creating the radar sensor AFTER the
# CDN wait ensures it is initialized close to when its render_product is
# created — if the radar prim exists for 140+ seconds before the render_product
# is attached, the RTX plugin's internal state may expire.
lidar_path = attach_lidar_sensor(stage, robot_path)

# Flush after the (possibly long) lidar CDN wait before creating the radar.
simulation_app.update()

radar_path = attach_radar_sensor(stage, robot_path)

# Second flush: give the radar prim one render pass before attaching annotator.
simulation_app.update()

# Set up radar annotator + UDP send socket
radar_annotator = None
_radar_sock = None
_radar_udp_addr = None

if radar_path:
    radar_annotator = setup_radar_annotator(radar_path)
    carb.log_warn(f"[diag] radar_annotator={radar_annotator!r}")

    _rdr_cfg = load_config("radar_params.yaml")
    _udp = _rdr_cfg.get("udp", {})
    _radar_udp_addr = (_udp.get("remote_ip", "239.0.0.1"), int(_udp.get("remote_port", 10001)))
    try:
        _radar_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        _radar_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
        carb.log_warn(f"[diag] Radar UDP socket ready, sending to {_radar_udp_addr}")
    except OSError as _e:
        carb.log_warn(f"Could not create radar UDP socket: {_e}")

# Set up lidar annotator + UDP send socket (same pipeline as radar)
lidar_annotator = None
_lidar_sock = None
_lidar_udp_addr = None
_lidar_udp_sent = 0

if lidar_path and not args.no_ros2:
    lidar_annotator = setup_lidar_annotator(stage, lidar_path)
    if lidar_annotator is not None:
        _ldr_cfg = load_config("lidar_params.yaml")
        _ludp = _ldr_cfg.get("udp", {})
        _lidar_udp_addr = (
            _ludp.get("remote_ip", "239.0.0.1"),
            int(_ludp.get("remote_port", 10002)),
        )
        try:
            _lidar_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            _lidar_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
            carb.log_warn(f"[diag] Lidar UDP socket ready, sending to {_lidar_udp_addr}")
        except OSError as _e:
            carb.log_warn(f"Could not create lidar UDP socket: {_e}")
elif args.no_ros2:
    carb.log_info("--no-ros2: skipping LiDAR annotator")

if args.weather != "clear":
    add_weather_effects(stage, weather_type=args.weather, fog_preset=args.fog_preset)

if args.dynamic_objects:
    add_dynamic_objects(stage)

# ── Start timeline ───────────────────────────────────────────────────────────
timeline = omni.timeline.get_timeline_interface()
timeline.play()
carb.log_info("=== Simulation running — send SIGINT or SIGTERM to stop cleanly ===")

_LOG_EVERY = 100  # frames between status log lines
start_time = time.time()
frame = 0
_udp_sent = 0
_data_none_count = 0
_data_empty_count = 0
_lidar_empty = 0   # frames where lidar annotator returned empty data

try:
    while simulation_app.is_running() and not _shutdown:
        simulation_app.update()
        frame += 1

        # Parse GMO annotator data and send structured RDR2 packet via UDP.
        #
        # get_gmo_data() (from isaacsim.sensors.rtx) parses the raw OMGN bytes
        # into a Python object; we then extract x/y/z/velocity/scalar and pack
        # a simple RDR2 binary frame to avoid shipping OMGN internals over UDP.
        #
        # RDR2 wire format (little-endian):
        #   Header  16 bytes: magic(4s) + num_detections(I) + timestamp_ns(Q)
        #   Points  N * 24 bytes: [x, y, z, velocity, rcs, snr] each float32
        #
        # Falls back to raw OMGN bytes if get_gmo_data is unavailable.
        if radar_annotator is not None and _radar_sock is not None:
            _rdata = radar_annotator.get_data()
            if _rdata is None:
                _data_none_count += 1
            else:
                _raw = _rdata.get("data") if isinstance(_rdata, dict) else _rdata
                if _raw is None or len(_raw) == 0:
                    _data_empty_count += 1
                elif _get_gmo_data is not None:
                    try:
                        gmo = _get_gmo_data(_raw)
                        if gmo is None or gmo.magicNumber != _GMO_MAGIC:
                            _data_empty_count += 1
                        elif gmo.numElements == 0:
                            _data_empty_count += 1
                        else:
                            n = int(gmo.numElements)
                            ts = int(gmo.timestampNs) if gmo.timestampNs is not None else 0

                            # Log first valid frame — confirm coord type and all field values.
                            # OmniRadar WpmDmatApprox outputs spherical coords:
                            #   gmo.x = azimuth (deg), gmo.y = elevation (deg), gmo.z = range (m)
                            if _udp_sent == 0:
                                carb.log_warn(f"[GMO first frame] numElements={n}")
                                for _attr in ("x", "y", "z", "scalar", "velocities",
                                              "elementsCoordsType", "frameOfReference"):
                                    _v = getattr(gmo, _attr, None)
                                    _sh = _v.shape if hasattr(_v, "shape") else type(_v).__name__
                                    _s = _v[:3] if hasattr(_v, "__len__") and len(_v) >= 3 else _v
                                    carb.log_warn(f"  gmo.{_attr}: shape={_sh} sample={_s}")
                                # Print all detection coords for sanity check
                                carb.log_warn("  All detections (az_deg, el_deg, range_m, scalar):")
                                for _i in range(n):
                                    carb.log_warn(
                                        f"    [{_i:2d}] az={gmo.x[_i]:8.3f} el={gmo.y[_i]:7.3f}"
                                        f" r={gmo.z[_i]:7.3f} rcs={gmo.scalar[_i]:7.3f}"
                                    )

                            # Convert spherical (az_deg, el_deg, range_m) → Cartesian (x, y, z)
                            # in the sensor frame (x=forward, y=left, z=up).
                            az_rad = np.deg2rad(np.asarray(gmo.x, dtype=np.float64))
                            el_rad = np.deg2rad(np.asarray(gmo.y, dtype=np.float64))
                            r = np.asarray(gmo.z, dtype=np.float64)
                            cart_x = (r * np.cos(el_rad) * np.cos(az_rad)).astype(np.float32)
                            cart_y = (r * np.cos(el_rad) * np.sin(az_rad)).astype(np.float32)
                            cart_z = (r * np.sin(el_rad)).astype(np.float32)

                            rcs = (np.asarray(gmo.scalar, dtype=np.float32)
                                   if gmo.scalar is not None else np.zeros(n, np.float32))

                            vel_raw = gmo.velocities
                            if vel_raw is not None and len(vel_raw) == n:
                                vel = np.asarray(vel_raw, dtype=np.float32)
                                velocity = vel[:, 0] if vel.ndim == 2 else vel
                            else:
                                velocity = np.zeros(n, np.float32)

                            snr = np.zeros(n, np.float32)

                            header = struct.pack("<4sIQ", b"RDR2", n, ts)
                            points = np.column_stack(
                                [cart_x, cart_y, cart_z, velocity, rcs, snr]
                            ).astype(np.float32).tobytes()
                            _radar_sock.sendto(header + points, _radar_udp_addr)
                            _udp_sent += 1
                    except Exception as _exc:
                        carb.log_warn(f"[radar UDP send error] {type(_exc).__name__}: {_exc}")
                else:
                    # Fallback: send raw OMGN bytes (udp_listener will fail to parse but won't crash)
                    try:
                        _radar_sock.sendto(
                            _raw.tobytes() if hasattr(_raw, "tobytes") else bytes(_raw),
                            _radar_udp_addr,
                        )
                        _udp_sent += 1
                    except Exception as _exc:
                        carb.log_warn(f"[radar UDP fallback send error] {type(_exc).__name__}: {_exc}")

        # Duration-based exit
        if args.duration > 0.0 and (time.time() - start_time) >= args.duration:
            carb.log_info(f"--duration {args.duration:.1f}s reached — stopping")
            break

        # Parse lidar GMO data and send LDR2 UDP packet to ros2-bridge.
        # LDR2 wire format is identical to RDR2 (same struct, different magic):
        #   Header  16 bytes: magic(b'LDR2') + num_points(I) + timestamp_ns(Q)
        #   Points  N * 24 bytes: [x, y, z, intensity, 0, 0] each float32
        # OmniLidar OMGN data may be Cartesian (direct xyz) or spherical;
        # log elementsCoordsType on first frame to determine conversion needed.
        if lidar_annotator is not None and _lidar_sock is not None and _get_gmo_data is not None:
            try:
                _ldata = lidar_annotator.get_data()
                _lraw = _ldata.get("data") if isinstance(_ldata, dict) else _ldata
                if _lraw is None or len(_lraw) == 0:
                    _lidar_empty += 1
                else:
                    lgmo = _get_gmo_data(_lraw)
                    if lgmo is None or lgmo.magicNumber != _GMO_MAGIC or lgmo.numElements == 0:
                        _lidar_empty += 1
                    else:
                        ln = int(lgmo.numElements)
                        lts = int(lgmo.timestampNs) if lgmo.timestampNs is not None else 0

                        # Log first valid lidar frame to confirm coord type
                        if _lidar_udp_sent == 0:
                            carb.log_warn(
                                f"[lidar GMO first frame] numElements={ln} "
                                f"coordsType={lgmo.elementsCoordsType} "
                                f"frame={lgmo.frameOfReference}"
                            )
                            carb.log_warn(
                                f"  x[:3]={np.asarray(lgmo.x)[:3]} "
                                f"y[:3]={np.asarray(lgmo.y)[:3]} "
                                f"z[:3]={np.asarray(lgmo.z)[:3]}"
                            )

                        # OmniLidar OMGN format: check elementsCoordsType.
                        # If SPHERICAL: x=az_deg, y=el_deg, z=range_m → convert.
                        # If CARTESIAN: x,y,z are already in sensor frame.
                        _lx = np.asarray(lgmo.x, dtype=np.float64)
                        _ly = np.asarray(lgmo.y, dtype=np.float64)
                        _lz = np.asarray(lgmo.z, dtype=np.float64)
                        try:
                            from isaacsim.sensors.rtx import CoordsType
                            _is_spherical = (lgmo.elementsCoordsType == CoordsType.SPHERICAL)
                        except Exception:
                            _is_spherical = False  # assume Cartesian if can't check

                        if _is_spherical:
                            _az_r = np.deg2rad(_lx)
                            _el_r = np.deg2rad(_ly)
                            _r = _lz
                            lx32 = (_r * np.cos(_el_r) * np.cos(_az_r)).astype(np.float32)
                            ly32 = (_r * np.cos(_el_r) * np.sin(_az_r)).astype(np.float32)
                            lz32 = (_r * np.sin(_el_r)).astype(np.float32)
                        else:
                            lx32 = _lx.astype(np.float32)
                            ly32 = _ly.astype(np.float32)
                            lz32 = _lz.astype(np.float32)

                        lint = (np.asarray(lgmo.scalar, dtype=np.float32)
                                if lgmo.scalar is not None else np.zeros(ln, np.float32))
                        lzero = np.zeros(ln, np.float32)
                        all_points = np.column_stack(
                            [lx32, ly32, lz32, lint, lzero, lzero]
                        ).astype(np.float32)
                        # Filter out zero-range points (invalid returns)
                        # _lz holds range_m (spherical) or z (cartesian)
                        if _is_spherical:
                            valid_mask = (_lz != 0.0)
                        else:
                            valid_mask = (np.sqrt(_lx**2 + _ly**2 + _lz**2) > 0.01)
                        all_points = all_points[valid_mask]
                        n_valid = len(all_points)
                        # Chunk into UDP-safe packets (~2700 pts × 24B = 64800B < 65507B limit)
                        _MAX_PTS_PER_PKT = 2700
                        for _chunk_start in range(0, max(n_valid, 1), _MAX_PTS_PER_PKT):
                            _chunk = all_points[_chunk_start: _chunk_start + _MAX_PTS_PER_PKT]
                            _cn = len(_chunk)
                            if _cn == 0:
                                break
                            lheader = struct.pack("<4sIQ", b"LDR2", _cn, lts)
                            _lidar_sock.sendto(lheader + _chunk.tobytes(), _lidar_udp_addr)
                        _lidar_udp_sent += 1
                        if _lidar_udp_sent == 1:
                            _n_pkts = max(1, (n_valid + _MAX_PTS_PER_PKT - 1) // _MAX_PTS_PER_PKT)
                            carb.log_warn(
                                f"[lidar UDP] first frame: {ln} raw, {n_valid} valid pts"
                                f" → {_n_pkts} UDP packets"
                            )
            except Exception as _lexc:
                carb.log_warn(f"[lidar UDP send error] {type(_lexc).__name__}: {_lexc}")

        # Periodic status line
        if frame % _LOG_EVERY == 0:
            elapsed = time.time() - start_time
            radar_ok = bool(radar_path and stage.GetPrimAtPath(radar_path).IsValid())
            # lidar_path is now the OmniLidar sensor prim path (.../LidarSensor/sensor)
            lidar_ok = bool(lidar_path and stage.GetPrimAtPath(lidar_path).IsValid())
            carb.log_warn(
                f"[frame {frame:6d} | {elapsed:8.1f}s"
                f" | radar_ok={radar_ok} | lidar_ok={lidar_ok}"
                f" | udp_sent={_udp_sent} none={_data_none_count} empty={_data_empty_count}"
                f" | lidar_udp={_lidar_udp_sent} lidar_empty={_lidar_empty}]"
            )

finally:
    carb.log_info("=== Stopping timeline and closing Isaac Sim ===")
    timeline.stop()
    simulation_app.close()
    carb.log_info("=== Isaac Sim closed cleanly ===")
