# Isaac Sim Radar — Execution Plan

> **Created**: 2026-02-17
> **Last Updated**: 2026-02-23
> **Status**: In Progress — Pivoting to Docker

---

## Architecture

**Multi-container docker-compose setup:**

| Container | Base Image | Purpose |
|---|---|---|
| `isaac-sim` | `nvcr.io/nvidia/isaac-sim:5.1.0` (Ubuntu 22.04) | Headless Isaac Sim with radar/lidar sensors, scene loading |
| `ros2-bridge` | Ubuntu 22.04 + ROS2 Humble | Radar UDP bridge, RViz2, TF publishers, analysis tools |

Communication: `--network host` for ROS2 DDS discovery + UDP radar data.
GPU: NVIDIA Container Toolkit, GPU passed to `isaac-sim` container.

### Version Choices
- **Isaac Sim 5.1.0** — latest GA release (Oct 2025), best sensor support
- **Ubuntu 22.04** — in containers, matches NVIDIA's base image
- **ROS2 Humble** — LTS, native to Ubuntu 22.04, well-tested with Isaac Sim 5.x
- **Driver**: 580.65.06+ required on host for Isaac Sim 5.x

---

## Completed Work (Pre-Docker)

These steps produced working code that carries forward into the Docker setup:

- [x] **Step 2**: Project structure & ROS2 workspace (`ros2_ws/src/radar_bridge/`)
- [x] **Step 3**: Environment setup scripts (`setup_env.sh`, `requirements.txt`)
- [x] **Step 4**: Radar-to-ROS2 bridge node (`udp_listener.py`, `radar_to_ros2.py`)
- [x] **Step 5**: Isaac Sim scripts (`launch_scene.py`, `enable_extensions.py`, `robot_teleop.py`)
- [x] **Step 6**: Config, launch, RViz & analysis files
- [x] **Step 7**: Unit tests — 23/23 passing
- [x] ROS2 Jazzy installed on host (useful for host-side debugging)

---

## Step 8: Docker Infrastructure

### 8a. Host Prerequisites
- [ ] Install NVIDIA Container Toolkit (`nvidia-ctk`)
- [ ] Verify: `docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi`
- [ ] Authenticate with NGC: `docker login nvcr.io` (requires NGC API key)
- [ ] Upgrade GPU driver to 580+ if targeting remote compute with Isaac Sim 5.1

### 8b. Isaac Sim Container (`docker/isaac-sim/`)
- [ ] `Dockerfile` — extends `nvcr.io/nvidia/isaac-sim:5.1.0`
  - Copy `isaac_sim_scripts/` and `config/` into container
  - Install any additional Python deps
  - Set entrypoint for headless operation
- [ ] Verify: container starts, GPU detected, extensions load
- [ ] Update `launch_scene.py` for Isaac Sim 5.1 API changes (if any)

### 8c. ROS2 Bridge Container (`docker/ros2-bridge/`)
- [ ] `Dockerfile` — Ubuntu 22.04 + ROS2 Humble
  - Install `ros-humble-desktop`, sensor-msgs, tf2-ros, slam-toolbox
  - Copy `ros2_ws/` and build with colcon
  - Copy `radar_analysis/`, `launch/`, `rviz/`, `config/`
  - Install Python deps (numpy, open3d, matplotlib, scipy)
- [ ] Verify: `ros2 run radar_bridge radar_to_ros2` works inside container

### 8d. Docker Compose (`docker-compose.yaml`)
- [ ] Define `isaac-sim` service:
  - `nvcr.io/nvidia/isaac-sim:5.1.0` base
  - GPU runtime, `--network host`
  - Volume mount for scene USD files and config
  - Headless entrypoint
- [ ] Define `ros2-bridge` service:
  - Custom ROS2 Humble image
  - `--network host` for DDS + UDP
  - Volume mount for bags output and analysis results
  - Display passthrough for RViz2 (`DISPLAY`, `/tmp/.X11-unix`)
- [ ] Shared volumes: `config/`, `bags/`, scene assets
- [ ] Convenience scripts: `docker/start.sh`, `docker/stop.sh`

---

## Step 9: Update Code for Isaac Sim 5.1

- [ ] Review Isaac Sim 5.1 API changes vs 4.5 (sensor creation, OmniGraph nodes)
- [ ] Update `launch_scene.py` for any breaking changes
- [ ] Update `enable_extensions.py` extension names if changed
- [ ] Update `radar_params.yaml` if WpmDmatApproxRadar config format changed
- [ ] Re-test with containerized Isaac Sim

---

## Step 10: Integration Testing

- [ ] `docker compose up` → both containers start cleanly
- [ ] Isaac Sim loads scene, sensors active, radar emitting UDP
- [ ] ROS2 bridge receives UDP, publishes PointCloud2 on `/radar/point_cloud`
- [ ] `ros2 topic list` shows radar and lidar topics
- [ ] RViz2 displays both point clouds side-by-side
- [ ] Record a test bag file via `radar_analysis/record_bags.py`

---

## Remaining Project Milestones (from PROJECT_PLAN.md)

- [ ] **M1**: Urban scene + controllable robot with sensors
- [ ] **M2**: Radar data visible in RViz2
- [ ] **M3**: LiDAR data visible, side-by-side with radar
- [ ] **M4**: Recorded bags, offline comparison metrics
- [ ] **M5**: Analysis report (coverage, accuracy, detection density)
- [ ] **M6**: Material-aware radar tuning
- [ ] **M7**: Final documentation and reproducibility

---

## File Structure (Docker additions)

```
isaac-sim-radar/
├── docker/
│   ├── docker-compose.yaml
│   ├── isaac-sim/
│   │   └── Dockerfile
│   ├── ros2-bridge/
│   │   └── Dockerfile
│   ├── start.sh
│   └── stop.sh
├── ros2_ws/                    # (existing — mounted into ros2-bridge)
├── isaac_sim_scripts/          # (existing — mounted into isaac-sim)
├── config/                     # (existing — shared volume)
├── radar_analysis/             # (existing — in ros2-bridge)
├── launch/                     # (existing — in ros2-bridge)
├── rviz/                       # (existing — in ros2-bridge)
├── tests/                      # (existing — run in ros2-bridge or host)
└── ...
```
