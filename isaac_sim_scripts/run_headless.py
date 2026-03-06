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

simulation_app = SimulationApp({"headless": True, "anti_aliasing": 0})

# ── Safe to import omni.* now ───────────────────────────────────────────────
import carb  # noqa: E402

# WpmDmatApprox radar requires MotionBVH (ray-BVH for moving objects), which is
# not initialised in headless mode.  NVIDIA provides this explicit bypass flag.
# Without it the plugin calls std::terminate() when the first OmniRadar prim
# is instantiated.  The bypass disables motion-blur-accurate multi-bounce traces
# for dynamic objects but the static scene accuracy is unaffected.
carb.settings.get_settings().set("/app/sensors/nv/radar/runWithoutMBVH", True)
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import UsdGeom  # noqa: E402

from enable_extensions import enable_extensions  # noqa: E402
from launch_scene import (  # noqa: E402
    add_dynamic_objects,
    add_weather_effects,
    attach_lidar_sensor,
    attach_radar_sensor,
    create_ground_plane,
    create_urban_environment,
    load_config,
    setup_lidar_ros2_publisher,
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

# ── Build scene ──────────────────────────────────────────────────────────────
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

carb.log_info("=== Building urban scene ===")
create_ground_plane(stage)
create_urban_environment(stage)
robot_path = spawn_robot(stage)
radar_path = attach_radar_sensor(stage, robot_path)
lidar_path = attach_lidar_sensor(stage, robot_path)

# Set up radar annotator + UDP send socket
radar_annotator = None
_radar_sock = None
_radar_udp_addr = None

if radar_path:
    radar_annotator = setup_radar_annotator(radar_path)

    _rdr_cfg = load_config("radar_params.yaml")
    _udp = _rdr_cfg.get("udp", {})
    _radar_udp_addr = (_udp.get("remote_ip", "239.0.0.1"), int(_udp.get("remote_port", 10001)))
    try:
        _radar_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        _radar_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
        carb.log_info(f"Radar UDP socket ready, sending to {_radar_udp_addr}")
    except OSError as _e:
        carb.log_warn(f"Could not create radar UDP socket: {_e}")

if lidar_path and not args.no_ros2:
    setup_lidar_ros2_publisher(stage, lidar_path)
elif args.no_ros2:
    carb.log_info("--no-ros2: skipping LiDAR ROS2 publisher")

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

try:
    while simulation_app.is_running() and not _shutdown:
        simulation_app.update()
        frame += 1

        # Send raw GMO bytes from OmniRadar annotator via UDP each frame
        if radar_annotator is not None and _radar_sock is not None:
            _rdata = radar_annotator.get_data()
            if _rdata is not None:
                _raw = _rdata.get("data") if isinstance(_rdata, dict) else _rdata
                if _raw is not None and len(_raw) > 0:
                    try:
                        _radar_sock.sendto(bytes(_raw), _radar_udp_addr)
                    except OSError:
                        pass

        # Duration-based exit
        if args.duration > 0.0 and (time.time() - start_time) >= args.duration:
            carb.log_info(f"--duration {args.duration:.1f}s reached — stopping")
            break

        # Periodic status line
        if frame % _LOG_EVERY == 0:
            elapsed = time.time() - start_time
            radar_ok = bool(radar_path and stage.GetPrimAtPath(radar_path).IsValid())
            lidar_ok = bool(lidar_path and stage.GetPrimAtPath(lidar_path).IsValid())
            carb.log_info(
                f"[frame {frame:6d} | {elapsed:8.1f}s"
                f" | radar_ok={radar_ok} | lidar_ok={lidar_ok}]"
            )

finally:
    carb.log_info("=== Stopping timeline and closing Isaac Sim ===")
    timeline.stop()
    simulation_app.close()
    carb.log_info("=== Isaac Sim closed cleanly ===")
