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

### 8b. Isaac Sim Container (`docker/isaac-sim/`) ✅
- [x] `Dockerfile` — extends `nvcr.io/nvidia/isaac-sim:5.1.0`, pyyaml installed, entrypoint set
- [x] `entrypoint.sh` — headless by default, passthrough args for one-off commands

### 8c. ROS2 Bridge Container (`docker/ros2-bridge/`) ✅
- [x] `Dockerfile` — `osrf/ros:humble-desktop`, tf2-ros, slam-toolbox, colcon build at image build time
- [x] `entrypoint.sh` — sources ROS2, rebuilds on bind-mount changes, sources overlay

### 8d. Docker Compose + Scripts ✅
- [x] `docker/docker-compose.yaml` — both services, host networking, GPU runtime, 7 named cache volumes, healthchecks
- [x] `docker/start.sh` — pre-flight checks (GPU, NGC, X11), `docker compose up --build -d`
- [x] `docker/stop.sh` — `docker compose down`, revoke X11, `--clean` removes volumes
- [x] `docker/.env` — defaults for ROS_DOMAIN_ID, COMPOSE_PROJECT_NAME

---

## Step 9: Update Code for Isaac Sim 5.1 ✅

- [x] Reviewed Isaac Sim 5.1 API changes vs 4.5 — extension renames, OmniGraph node type strings
- [x] Updated `launch_scene.py` — Nucleus path `4.5→5.1`, node types `isaacsim.*`, defensive radar attr loop
- [x] Updated `enable_extensions.py` — `omni.isaac.ros2_bridge→isaacsim.ros2.bridge`, `omni.isaac.sensor→isaacsim.sensors.rtx`
- [x] Updated `radar_params.yaml` — comment updated to 5.1, OmniRadar prim note added
- [ ] Re-test with containerized Isaac Sim (requires NGC auth + GPU)

---

## Step 10: Integration Testing

### 10a. Pre-container testing ✅
- [x] `tests/test_integration.py` — 8 tests: full UDP→parse→pack roundtrip, PointCloud2 field layout, zero-detection edge case
- [x] `docker/test.sh` — post-start smoke test: GPU, pkg list, pytest, imports, node startup
- [x] `docker/docker-compose.yaml` — ros2-bridge healthcheck added (ros2 pkg list gate)

### 10b. Container runtime (requires NGC auth + GPU)
- [ ] `docker/start.sh` → both containers healthy (`docker ps` shows `(healthy)`)
- [ ] `docker/test.sh` → all 5 checks pass
- [ ] Isaac Sim loads scene: `docker exec isaac-sim /isaac-sim/python.sh /app/isaac_sim_scripts/launch_scene.py`
- [ ] Radar emitting UDP: `docker exec ros2-bridge ros2 topic list` shows `/radar/point_cloud`
- [ ] RViz2 displays both point clouds side-by-side
- [ ] Record test bag: `docker exec ros2-bridge python3 /app/radar_analysis/record_bags.py --duration 30`

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
