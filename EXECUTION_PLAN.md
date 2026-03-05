# Isaac Sim Radar — Execution Plan

> **Created**: 2026-02-17
> **Last Updated**: 2026-03-05
> **Status**: In Progress — Infrastructure complete; RTX radar headless crash identified; Xvfb fix in progress

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
- [x] **Step 7**: Unit tests — 23/23 passing (now 38/40; 2 open3d-import skips)
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
- [ ] Isaac Sim loads scene: `docker exec isaac-sim /isaac-sim/python.sh /app/isaac_sim_scripts/run_headless.py`
- [ ] Radar emitting UDP: `docker exec ros2-bridge ros2 topic list` shows `/radar/point_cloud`
- [ ] RViz2 displays both point clouds side-by-side
- [ ] Record test bag: `docker exec ros2-bridge python3 /app/radar_analysis/record_bags.py --duration 30`

---

## Step 11: Urban Scene + Offline Analysis Pipeline ✅

### 11a. Urban scene geometry ✅
- [x] `launch_scene.py` — `create_urban_environment(stage)` added: 4 concrete buildings, 2 walls, 3 metal pillars with material bindings and collision APIs

### 11b. Offline analysis pipeline ✅
- [x] `radar_analysis/run_analysis.py` — reads rosbag2 files via pure-Python `rosbags`, computes metrics, saves 4 plots + Markdown report
- [x] `radar_analysis/parse_radar_udp.py` — fixed fragile relative `sys.path`
- [x] `requirements.txt` — added `rosbags>=0.9`
- [x] `tests/test_run_analysis.py` — 9 new tests (synthetic clouds, headless `Agg` backend)

**Test count: 38/40 passing (2 open3d import skips)**

---

## Step 12: Headless Simulation Runner ✅

### 12a. `isaac_sim_scripts/run_headless.py` ✅
- [x] `SimulationApp`-first entry point: boots Isaac Sim headless, calls `enable_extensions`, calls `create_urban_environment` + sensor setup from `launch_scene.py`
- [x] Timeline play + update loop (`while app.is_running()`)
- [x] SIGINT/SIGTERM handler → clean `app.close()`
- [x] CLI: `--duration N` (0 = run until signal), `--no-ros2` (skip LiDAR publisher)
- [x] Status log every 100 frames: `[frame N | T.Ts | radar_ok | lidar_ok]`

### 12b. `docker/isaac-sim/entrypoint.sh` ✅
- [x] Changed default command from `runheadless.native.sh` to `run_headless.py`
  - `docker compose up` now automatically loads the scene on container start

---

## Step 13: SLAM Toolbox + Nav2 Integration ✅

### 13a. ROS2 bridge Dockerfile ✅
- [x] Added `ros-humble-pointcloud-to-laserscan` — converts `/lidar/point_cloud` (PointCloud2) → `/scan` (LaserScan) needed by SLAM Toolbox
- [x] Added `ros-humble-navigation2` + `ros-humble-nav2-bringup` for autonomous navigation

### 13b. `config/slam_toolbox.yaml` ✅
- [x] Online sync mapping mode; 5 cm resolution tuned for OS1-64 at urban scale
- [x] `use_sim_time: true` throughout; Ceres solver; loop closure enabled
- [x] Input: `/scan` (from pointcloud_to_laserscan)

### 13c. `config/nav2_params.yaml` ✅
- [x] DWB local planner (max 0.5 m/s), NavFn global planner, Spin/BackUp recoveries
- [x] Global + local costmaps (5 cm resolution, 0.8 m inflation for urban buildings)
- [x] AMCL config (for localization mode when loading a pre-built map)
- [x] All nodes: `use_sim_time: true`

### 13d. `launch/slam.launch.py` ✅
- [x] `pointcloud_to_laserscan` node (horizontal slice ±0.5 m, 10 Hz, 720 rays/scan)
- [x] `slam_toolbox sync_slam_toolbox_node` wired to `/scan`
- [x] `use_sim_time` arg (default true)

### 13e. `launch/nav2.launch.py` ✅
- [x] controller_server, planner_server, recoveries_server, bt_navigator
- [x] lifecycle_manager with autostart
- [x] `params_file` arg for runtime override

### 13f. `docker/test.sh` ✅
- [x] Updated next-steps hints to reference `run_headless.py` entrypoint, SLAM, and Nav2 launches

---

## Step 14: Radar Occupancy Map + Sensor Fusion ✅

### 14a. `radar_analysis/radar_map.py` ✅
- [x] `RadarOccupancyGrid` — 2D accumulation grid; configurable resolution, extents, RCS threshold
- [x] `accumulate()` — transforms detections from sensor→odom frame via (x, y, yaw); returns count
- [x] `save_map()` — writes PNG heat map + ROS PGM + ROS YAML (resolution, origin)
- [x] `build_from_bag()` — reads rosbag2, syncs radar↔odom by nearest timestamp, builds and saves grid
- [x] Standalone CLI: `python3 radar_map.py <bag> --out <dir>`

### 14b. `radar_analysis/sensor_fusion.py` ✅
- [x] `FusionFrame` dataclass — per-frame metrics: detection_ratio, coverage_complement_m, coverage_pct_covered, mean_radar_velocity
- [x] `fuse_pair()` — vectorised 2D nearest-neighbour; configurable coverage threshold
- [x] `save_metrics_csv()` — per-frame metrics CSV with nan handling
- [x] `plot_fusion_summary()` — 3-panel matplotlib figure (ratio, complement distance, coverage %)
- [x] `run_from_bag()` — syncs radar↔lidar by nearest timestamp (±100 ms), writes CSV + plot
- [x] Standalone CLI: `python3 sensor_fusion.py <bag> --out <dir>`

### 14c. Tests ✅
- [x] `tests/test_radar_map.py` — 15 tests: grid shape, identity/translation/rotation transforms, RCS filtering, out-of-bounds, multi-frame accumulation, save file formats
- [x] `tests/test_sensor_fusion.py` — 16 tests: identical/disjoint/empty clouds, detection ratio, velocity mean/abs, CSV output, nan handling, plot save

**Test count: 69/71 passing (2 pre-existing open3d import skips)**

---

## Step 15: Scenario Variations ✅

### 15a. `isaac_sim_scripts/launch_scene.py` ✅
- [x] `add_weather_effects(stage, weather_type, fog_preset)`:
  - `fog` — sets RTX fog settings via `carb.settings` (degrades LiDAR range; radar unaffected)
  - `rain` — scatters 60 metallic sphere prims throughout the scene volume (creates radar clutter)
  - `clear` — no-op
- [x] `add_dynamic_objects(stage, num_pedestrians, num_vehicles, num_frames)`:
  - Pedestrians: cylinders on triangle-wave oscillating paths, USD time-sampled animation baked for `num_frames`
  - Vehicles: car-sized boxes on one-way linear paths, USD time-sampled animation baked
  - Animation plays automatically as the timeline advances — no update-loop changes needed

### 15b. `isaac_sim_scripts/run_headless.py` ✅
- [x] `--weather {clear,fog,rain}` arg (default: clear)
- [x] `--fog-preset {light,dense}` arg (default: light)
- [x] `--dynamic-objects` flag
- [x] Calls `add_weather_effects` and `add_dynamic_objects` after scene setup

### 15c. Tests ✅
- [x] `tests/test_scenario_variations.py` — 29 tests:
  - Triangle-wave interpolation (monotone, symmetric, boundary values)
  - Pedestrian/vehicle position bounds and path correctness
  - Fog preset table structure and ordering
  - CLI argparse: all new flags, invalid value rejection, combined flags

**Test count: 98/100 passing (2 pre-existing open3d import skips)**

---

---

## Step 16: RTX Radar Headless Crash — Diagnosis & Fix

### 16a. Root Cause ❌ (identified 2026-03-05)

When running Isaac Sim 5.1 headless via `python.sh`, both RTX radar model plugins
(`wpm_dmatapprox` and `dmatapprox`) crash in `carbOnPluginPreStartup`:

```
[Error] [carb.plugin] carbOnPluginPreStartup() failed for plugin wpm_dmatapprox
[Error] [carb.plugin] carbOnPluginPreStartup() failed for plugin dmatapprox
```

**Cause**: The carb interfaces these plugins require (`IMaterialReaderFactory`,
`IProfileReaderFactory`) are registered too late in the plugin startup sequence
when Isaac Sim is launched headless via `python.sh`. This is an **NVIDIA regression
in Isaac Sim 5.1** — the same plugins work in NVIDIA's own CI/CD pipelines, but
those use a different launch path.

**Note**: A physics-raycasting fallback was prototyped but discarded. The whole
point of using Isaac Sim is material-aware multi-bounce WPM radar simulation —
not synthetic raycasts.

### 16b. Proposed Fixes (evaluated 2026-03-05)

| Option | Likelihood | Effort | Notes |
|---|---|---|---|
| **Xvfb virtual framebuffer** | High | Low | `xvfb-run -a` changes carb plugin init order; NVIDIA docs recommend this for headless RTX |
| Downgrade to Isaac Sim 4.5 | High | Medium | Original target; WPM radar confirmed working in 4.5; requires re-testing API changes |
| Upgrade to Isaac Sim 5.2+ | Medium | Medium | Possible regression fix; no confirmed release date at time of writing |
| Use `isaac-sim.sh --exec` | Medium | Low | Full app stack pre-initializes all extensions before user script runs |

**Chosen**: Xvfb (Option 1) — 3-line Dockerfile change, zero code changes to simulation scripts.

### 16c. Xvfb Fix ⏳ (Step 16, in progress)

- [ ] `docker/isaac-sim/Dockerfile` — add `xvfb` package
- [ ] `docker/isaac-sim/entrypoint.sh` — wrap `python.sh` call with `xvfb-run -a`
- [ ] `docker/docker-compose.yaml` — remove any `DISPLAY` env workarounds if present
- [ ] Re-test: `docker run ... run_headless.py --no-ros2 --duration 30` — confirm no radar plugin crash
- [ ] Verify radar UDP packets emitted: `docker exec ros2-bridge ros2 topic echo /radar/point_cloud`

---

## Remaining Project Milestones (from PROJECT_PLAN.md)

- [~] **M1**: Urban scene geometry + headless runner complete in code; runtime verification pending (Step 10b)
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
│   ├── .env
│   ├── start.sh
│   ├── stop.sh
│   ├── test.sh                 # post-start smoke test (5 checks)
│   ├── isaac-sim/
│   │   ├── Dockerfile
│   │   └── entrypoint.sh
│   └── ros2-bridge/
│       ├── Dockerfile
│       └── entrypoint.sh
├── isaac_sim_scripts/          # mounted into isaac-sim container
│   ├── launch_scene.py         # scene + sensors (urban environment added Step 11)
│   ├── enable_extensions.py    # extension enable (updated for 5.1 in Step 9)
│   ├── robot_teleop.py
│   └── run_headless.py         # PLANNED Step 12 — SimulationApp boot + loop
├── ros2_ws/                    # mounted into ros2-bridge
│   └── src/radar_bridge/
│       ├── udp_listener.py
│       └── radar_to_ros2.py
├── radar_analysis/
│   ├── run_analysis.py         # NEW Step 11 — offline bag→metrics pipeline
│   ├── parse_radar_udp.py      # (path fix Step 11)
│   ├── compare_clouds.py
│   ├── visualize.py
│   ├── record_bags.py
│   ├── radar_pointcloud.py
│   └── lidar_pointcloud.py
├── config/
├── launch/
├── rviz/
├── tests/
│   ├── test_unit.py            # 23 original unit tests
│   ├── test_integration.py     # 8 tests added Step 10
│   └── test_run_analysis.py    # 9 tests added Step 11
└── requirements.txt            # rosbags>=0.9 added Step 11
```
