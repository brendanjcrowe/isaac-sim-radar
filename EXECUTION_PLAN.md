# Isaac Sim Radar — Execution Plan

> **Created**: 2026-02-17
> **Status**: In Progress

---

## Step 1: Install Prerequisites

### 1a. Install Omniverse Launcher & Isaac Sim 4.5
- [ ] Download Omniverse Launcher `.AppImage` from NVIDIA
- [ ] Install Isaac Sim 4.5 through the Launcher
- [ ] Verify launch and GPU detection
- *Manual/GUI step*

### 1b. Install ROS2 Jazzy
- [ ] Run `bash setup_env.sh` (handles apt repo, GPG key, package install)
- [ ] Verify: `source /opt/ros/jazzy/setup.bash && ros2 --help`

### 1c. Install Python Dependencies — DONE
- [x] Created conda environment `isaac-radar` (Python 3.12)
- [x] Installed: numpy, open3d, matplotlib, pyyaml, scipy, pytest

---

## Step 2: Project Structure & ROS2 Workspace — DONE

- [x] Created full directory tree
- [x] Created `ros2_ws/src/radar_bridge/` ROS2 package (package.xml, setup.py, setup.cfg, resource marker)
- [x] Created `isaac_sim_scripts/` directory
- [x] Created `radar_analysis/` directory
- [x] Created `config/`, `launch/`, `rviz/` directories

---

## Step 3: Environment Setup Script — DONE

- [x] `setup_env.sh` — installs ROS2 Jazzy, Python deps, builds workspace, prints Isaac Sim instructions
- [x] `requirements.txt` — numpy, open3d, matplotlib, pyyaml

---

## Step 4: Radar-to-ROS2 Bridge Node — DONE

- [x] `radar_bridge/udp_listener.py` — UDP multicast socket, GenericModelOutput binary parser
- [x] `radar_bridge/radar_to_ros2.py` — ROS2 node: UDP → PointCloud2 (`/radar/point_cloud`)
- [x] `launch/radar_bridge.launch.py` — parameterized launch file
- [x] `config/radar_bridge.yaml` — multicast group, port, frame_id, topic

---

## Step 5: Isaac Sim Scene Scripts — DONE

- [x] `isaac_sim_scripts/enable_extensions.py` — enable radar, lidar, ros2_bridge extensions
- [x] `isaac_sim_scripts/launch_scene.py` — load stage, spawn robot, attach sensors, configure OmniGraph
- [x] `isaac_sim_scripts/robot_teleop.py` — WASD keyboard teleop for differential drive

---

## Step 6: Config, Launch, RViz & Analysis — DONE

- [x] `config/radar_params.yaml` — WpmDmatApproxRadar parameters
- [x] `config/lidar_params.yaml` — OS1_64 LiDAR config
- [x] `config/robot_sensors.yaml` — TF frame tree with mount transforms
- [x] `launch/full_stack.launch.py` — master launch (bridge + TF + RViz2)
- [x] `rviz/radar_comparison.rviz` — radar (red) + LiDAR (green) side-by-side
- [x] `radar_analysis/parse_radar_udp.py` — standalone UDP capture tool
- [x] `radar_analysis/radar_pointcloud.py` — detections → Open3D point cloud
- [x] `radar_analysis/lidar_pointcloud.py` — LiDAR PCD loading + conversion
- [x] `radar_analysis/compare_clouds.py` — nearest-neighbor distance, coverage metrics
- [x] `radar_analysis/visualize.py` — BEV plots, histograms, RCS scatter
- [x] `radar_analysis/record_bags.py` — ROS2 bag recording wrapper

---

## Step 7: Unit Tests — DONE

- [x] `tests/test_udp_listener.py` — 10 tests: packet parsing, struct sizes, truncation handling, UDP send/receive
- [x] `tests/test_analysis.py` — 9 tests: distance metrics, coverage ratio, Open3D point cloud creation
- [x] `tests/test_config.py` — 4 tests: all YAML configs load with correct structure/values
- [x] All 23 tests passing (conda env `isaac-radar`, pytest)

---

## Verification Checklist

- [x] Unit tests pass: `conda activate isaac-radar && pytest tests/ -v` → 23/23 passed
- [ ] `bash setup_env.sh` → ROS2 Jazzy installed, workspace builds cleanly
- [ ] `ros2 run radar_bridge radar_to_ros2` → node starts, listens on UDP port
- [ ] Send mock UDP data → see PointCloud2 messages on `/radar/point_cloud`
- [ ] Open Isaac Sim → run `launch_scene.py` → scene loads, robot spawns, sensors active
- [ ] `ros2 topic list` shows radar and lidar topics
- [ ] RViz2 displays both point clouds
