# Isaac Sim mmWave Radar — Urban Mobile Robot Simulation

> **Status**: Active / Living Document
> **Created**: 2026-02-13
> **Last Updated**: 2026-03-30
> **Isaac Sim Version**: 5.1.0
> **Deployment**: Docker (multi-container)

---

## Document Conventions

- **[CHOSEN]** — The approach we are actively pursuing.
- **[ALTERNATIVE]** — A viable option we have documented but not selected. May revisit.
- **[DEPRECATED]** — An approach that was tried and abandoned, with rationale noted.
- Items marked with `TODO` require action.
- Decision history is preserved in the [Decision Log](#decision-log) at the bottom.

---

## Project Goal

Build a realistic city/town environment in NVIDIA Isaac Sim 5.1 to simulate a mobile robot platform equipped with a mmWave radar and complementary sensors. Traverse the environment, capture radar returns, display point clouds, and compare radar data against LiDAR point clouds/maps.

---

## Phase 0 — Prerequisites & Infrastructure

### 0.1 Hardware/Software Requirements

- **GPU**: NVIDIA RTX 3080+ (RTX 4090 or A6000 recommended for large urban scenes + multiple RTX sensors)
- **Host GPU Driver**: 580.65.06+ (required for Isaac Sim 5.x containers)
- **Docker** with **NVIDIA Container Toolkit** (`nvidia-ctk`)
- **NGC account** for pulling `nvcr.io/nvidia/isaac-sim:5.1.0`

### 0.2 Deployment Architecture **[CHOSEN]**

> **Decision**: Pivoted to Docker multi-container on 2026-02-23. Provides reproducibility, portability to different compute (local dev → cloud GPU), and clean separation of concerns.

**Multi-container docker-compose setup:**

| Container | Base Image | Role |
|---|---|---|
| `isaac-sim` | `nvcr.io/nvidia/isaac-sim:5.1.0` (Ubuntu 22.04) | Headless Isaac Sim — scene loading, physics, radar/lidar sensor simulation |
| `ros2-bridge` | Ubuntu 22.04 + ROS2 Humble | Radar UDP bridge → PointCloud2, RViz2, TF publishers, analysis tools |

- **Networking**: `--network host` for ROS2 DDS discovery + UDP radar multicast
- **GPU**: Passed to `isaac-sim` container via NVIDIA Container Toolkit
- **Volumes**: Shared `config/`, scene assets, bag recording output

### 0.3 Version Matrix

| Component | Version | Rationale |
|---|---|---|
| Isaac Sim | 5.1.0 | Latest GA (Oct 2025), best sensor/extension support |
| Ubuntu (containers) | 22.04 | NVIDIA's base image for Isaac Sim 5.x |
| ROS2 | Humble | LTS, native to Ubuntu 22.04, well-tested with Isaac Sim |
| Python | 3.10+ (container), 3.12 (host) | Container matches Isaac Sim's Python; host for analysis |

### 0.4 Enable Required Extensions

Extensions are enabled programmatically via `isaac_sim_scripts/enable_extensions.py` inside the `isaac-sim` container:

- `omni.sensors.nv.radar` — [Radar Extension](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/omni_sensors_docs/radar_extension.html) (Omniverse layer — unchanged in 5.x)
- `omni.sensors.nv.common` — GenericModelOutput format dependency (Omniverse layer — unchanged in 5.x)
- `isaacsim.ros2.bridge` — ROS2 integration (was `omni.isaac.ros2_bridge` in 4.5 — renamed in 5.x)
- `isaacsim.sensors.rtx` — RTX-based sensors (was `omni.isaac.sensor` in 4.5 — renamed in 5.x)
- `omni.sensors.nv.lidar` — RTX Lidar extension (Omniverse layer — unchanged in 5.x)

### 0.5 Previous Approach — Native Install **[DEPRECATED]**

> Replaced by Docker on 2026-02-23. Native install (Omniverse Launcher + host ROS2 Jazzy) had driver version conflicts between Isaac Sim 5.x (requires 580+) and the host system (535.288). Docker isolates this cleanly. Host ROS2 Jazzy install retained for local debugging.

### 0.6 Known Issue — RTX Radar Crash in Isaac Sim 5.1 Headless **[RESOLVED]**

> **Identified**: 2026-03-05 | **Resolved**: 2026-03-14

Both NVIDIA RTX radar model plugins (`wpm_dmatapprox`, `dmatapprox`) crash during
`carbOnPluginPreStartup` when Isaac Sim 5.1 is launched headless via `python.sh`.
The carb interfaces they depend on (`IMaterialReaderFactory`, `IProfileReaderFactory`)
are not yet registered when plugin startup runs in the `python.sh` launch path.

**Fix applied**: Manual Xvfb virtual framebuffer startup in `docker/isaac-sim/entrypoint.sh`
(not `xvfb-run -a`, which hangs in Docker due to SIGUSR1 delivery issues). This changes the
carb plugin initialization order so all required interfaces are registered before the radar
plugins start. See EXECUTION_PLAN.md Step 16c for details.

Additional data pipeline bugs were discovered and fixed during runtime verification
(Steps 16d–16i): sensor creation order, prim path handling, annotator selection, wire
format design, and UDP chunking. Full details in EXECUTION_PLAN.md.

---

## Phase 1 — Urban Environment Creation

Isaac Sim ships with **no built-in city/urban outdoor environments**. The bundled environments are indoor only (warehouse, hospital, office, simple rooms). We must build or source an urban scene.

### 1.1 Current Environment — Procedural Geometry **[ACTIVE]**

> Used for M1–M4 pipeline development and validation. Simple procedural geometry
> (4 buildings, 2 walls, 3 pillars) with correct material bindings. Sufficient
> for validating the full sensor pipeline; will be replaced with richer environments.

### 1.2 Option A — Video-to-3D Reconstruction **[CHOSEN for scaling]**

> **Decision**: Selected 2026-03-30 as the primary approach for scaling to diverse,
> photorealistic suburban environments. Will be pursued after M5–M7 are complete.

Use real-world video datasets to reconstruct 3D environments for Isaac Sim:

- **Source data**: KITTI-360 (calibrated stereo, suburban Karlsruhe), nuScenes, Waymo Open,
  or custom smartphone captures
- **Geometry pipeline**: COLMAP (pose estimation) → 3DGUT/Gaussian Splatting → USDZ import
  via Isaac Sim NuRec (supported in 5.0+), or COLMAP + OpenMVS → triangle mesh → USD
- **Material pipeline** (critical for radar): Semantic segmentation on source frames
  (RMSNet or SAM + CLIP) → project material labels onto mesh → map to Isaac Sim's ~40
  non-visual material types. Ref: "Material-informed Gaussian Splatting for Digital Twin"
  (arXiv 2511.20348) validates this approach for LiDAR reflectivity simulation
- **Collision meshes**: Extract via SuGaR (Gaussian → Poisson mesh) or use OpenMVS mesh directly

**Key risk**: No turnkey video-to-radar-simulatable-environment pipeline exists today.
The material assignment step requires custom engineering (semantic segmentation + projection +
mapping to Isaac Sim's `inputs:nonvisual:base` material bindings).

**Pros**: Photorealistic, grounded in real-world geometry, infinite suburban variety
**Cons**: Material assignment is a multi-week engineering effort with research risk

### 1.3 Option B — Procedural City Generation **[ALTERNATIVE]**

> Fallback if video reconstruction proves too labour-intensive for material assignment.
> Best "effort-to-radar-data" ratio since materials are correct by construction.

- Use **Blender + add-ons** (e.g., SceneCity, Buildify) to procedurally generate layouts, export USD
- Alternatively, **CesiumGS/cesium-omniverse** for real-world 3D tile streaming
- **Hybrid variant**: procedural layouts + NVIDIA SimReady assets for key objects (vehicles,
  street furniture) — best realism-to-effort ratio
- Pros: Scales infinitely, materials correct by construction, reproducible
- Cons: Looks procedural, requires Blender pipeline expertise

### 1.4 Option C — Assemble from Omniverse/Third-Party Assets **[ALTERNATIVE]**

- Use **NVIDIA Omniverse SimReady assets** + third-party FBX/OBJ from TurboSquid/Sketchfab
- Convert to USD, compose manually in Isaac Sim Composer
- Pros: Realistic individual assets with PBR materials
- Cons: Manual assembly doesn't scale; limited suburban-specific assets

### 1.5 Option D — NVIDIA Metropolis / Digital Twin Blueprint **[TRACKING]**

- The [NVIDIA Omniverse Blueprint for Smart Cities](https://blogs.nvidia.com/blog/smart-city-ai-blueprint-europe/) provides city-scale digital twin pipeline
- Not designed for robotics sim; tracking as ecosystem matures

### 1.4 Environment Setup Checklist (applies to all options)

- [ ] Stage set to **meters** and **Z-up**
- [ ] **Collisions enabled** on all geometry (required for PhysX Lidar and physics)
- [ ] **Physically-based materials** assigned — `WpmDmatApproxRadar` uses material-dependent radiometry, so correct materials (metal, concrete, glass, vegetation) directly affect radar fidelity
- [ ] Appropriate **lighting** (HDR sky dome for outdoor)
- [ ] **NavMesh** generated (if using autonomous navigation)

---

## Phase 2 — Mobile Robot Platform Setup

### 2.1 Select or Build a Robot

- **Bundled option**: [Idealworks iw.hub](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/assets/usd_assets_robots.html) (`Robots/Idealworks/iw_hub_sensors.usd`) — mobile base with LiDAR and cameras pre-configured
- **Custom option**: Import your own robot URDF/USD following [this workflow](https://manavthakkar.github.io/blog/taking-your-custom-mobile-robot-from-cad-to-isaac-sim) — define collision meshes, joints, drive controllers

### 2.2 Sensor Suite

| Sensor | Type | Purpose |
|---|---|---|
| mmWave Radar | RTX Radar (`WpmDmatApproxRadar`) | Primary sensor under study |
| LiDAR | RTX Lidar (e.g., Ouster OS1 config) | Ground-truth comparison |
| RGB Camera | Isaac Camera sensor | Visual context |
| IMU | PhysX IMU | Odometry / state estimation |
| GPS (optional) | Custom OmniGraph node | Global pose ground truth |

### 2.3 Radar Sensor Configuration

- Create the radar prim: **Create > Sensors > RTX Radar**
- Attach to a mount point on the robot chassis
- Choose model via `rtxsensor:modelConfig` attribute:
  - `DmatApproxRadar` — faster, suitable for initial testing
  - **`WpmDmatApproxRadar`** — recommended for realistic mmWave (multi-bounce, material-aware, antenna gain patterns)
- Customize the parameterization JSON in `data/sensors/radar/WpmDmatApproxRadar/`:

| Parameter Group | Key Parameters | Notes |
|---|---|---|
| Range | `MaxRangeM`, `RangeSepM`, `RangeResM` | Match to target radar spec |
| Velocity | `MaxVelMpsSequence`, `VelSepMps`, `VelResMps` | Doppler configuration |
| Angular FOV | `MaxAzAngDeg`, `MaxElevAngDeg` | Match antenna pattern |
| Resolution | Azimuth/elevation bore-sight separation | Controls angular resolution |
| CFAR | Test cells, guard cells, threshold | Detection sensitivity tuning |
| Noise | `RangeNoiseMean`, `RangeNoiseSDev`, `AzimuthRadNoiseSDev` | Match real sensor noise floor |

### 2.4 Navigation / Teleoperation

- Set up **differential-drive or Ackermann controller** via OmniGraph
- Add **keyboard teleop** for manual exploration
- Optionally integrate **Nav2** via ROS2 for autonomous waypoint following

---

## Phase 3 — Data Pipeline & ROS2 Integration

### 3.1 Radar Data Pipeline **[IMPLEMENTED — Custom UDP]**

> **Updated 2026-03-14**: `TranscoderRadar` OmniGraph node is for physical hardware (e.g.,
> CONTINENTAL ARS430) only — does not work with simulated RTX radar. Custom pipeline used instead.

```
[isaac-sim container]
RTX Radar (WpmDmatApproxRadar) → GenericModelOutput annotator → get_gmo_data() → RDR2 UDP (239.0.0.1:10001)
                                                                                     ↓ (host network)
[ros2-bridge container]
udp_listener.py → parse RDR2 → ROS2 PointCloud2 on /radar/point_cloud
```

- OmniRadar returns SPHERICAL coords (az_deg, el_deg, range_m); converted to Cartesian in-sim
- RDR2 wire format: 16-byte header + 24 bytes per detection (x, y, z, velocity, rcs, snr)
- 13–16 CFAR detections per frame at 97 Hz

### 3.2 Radar-to-ROS2 Bridge (Custom — Required) **[IMPLEMENTED]**

The radar extension does **not** natively publish to ROS2 topics. A bridge node is required:

**Approach 1 — UDP listener node [CHOSEN]**:
1. ~~Write a~~ ROS2 node (Python) listens on the UDP socket (default `239.0.0.1:10001`)
2. Parses the GenericModelOutput binary format (ref: `omni.sensors.nv.common` docs)
3. Publishes as `sensor_msgs/PointCloud2` (fields: x, y, z, velocity, RCS, SNR)
4. Configurable via YAML (port, frame_id, topic name)

> Implementation: `ros2_ws/src/radar_bridge/` — unit tested (38/40 passing; 2 open3d import skips)

**Approach 2 — Isaac Sim Python scripting [ALTERNATIVE]**:
- Read radar output buffers directly via the Isaac Sim Python API and publish via `rclpy`

### 3.3 LiDAR Data Pipeline **[IMPLEMENTED — Custom UDP]**

> **Updated 2026-03-20**: Isaac Sim's native OmniGraph `ROS2RtxLidarHelper` does not work
> for the OS1 CDN-based lidar model (`IsaacCreateRTXLidarScanBuffer` never populates).
> LiDAR uses the same custom pipeline as radar.

```
[isaac-sim container]
RTX Lidar (OS1-128) → GenericModelOutput annotator → get_gmo_data() → LDR2 UDP (239.0.0.1:10002)
                                                                          ↓ (host network)
[ros2-bridge container]
lidar_to_ros2.py → parse LDR2 → ROS2 PointCloud2 on /lidar/point_cloud
```

- OmniLidar returns ~43,776 points/frame (SPHERICAL coords); converted to Cartesian in-sim
- UDP packets chunked to 2700 pts each (OS1-128 full scan exceeds 64K UDP limit)
- `lidar_to_ros2` node in `ros2_ws/src/radar_bridge/` receives LDR2 and publishes PointCloud2

### 3.4 Frame / TF Configuration

- Publish TF tree: `map → odom → base_link → radar_link`, `base_link → lidar_link`, etc.
- Static TF publishers in `launch/full_stack.launch.py`
- Sensor mount offsets defined in `config/robot_sensors.yaml`

---

## Phase 4 — Visualization & Analysis

### 4.1 Real-Time Visualization (RViz2)

- **Radar point cloud**: Subscribe to radar PointCloud2 topic; color by RCS or velocity
- **LiDAR point cloud**: Subscribe to lidar PointCloud2 topic
- **Overlay both** in the same RViz2 session with different color maps for direct comparison
- Add robot model (URDF) visualization, TF frames, camera image panels
- Pre-configured layout: `rviz/radar_comparison.rviz`

### 4.2 Isaac Sim Viewport Visualization

- Enable RTX Radar point cloud rendering in viewport (via OmniGraph if supported)
- Enable RTX LiDAR viewport rendering for side-by-side comparison within the simulator

### 4.3 Offline Analysis Pipeline **[IMPLEMENTED]**

```
radar_analysis/
├── record_bags.py            # rosbag2 recording of all sensor topics
├── parse_radar_udp.py        # Parse raw GenericModelOutput binary files
├── radar_pointcloud.py       # Convert radar detections → Open3D point clouds
├── lidar_pointcloud.py       # Convert lidar scans → Open3D point clouds
├── compare_clouds.py         # Registration, ICP alignment, density comparison
└── visualize.py              # Matplotlib/Open3D 3D plots, heatmaps
```

### 4.4 Key Analyses

- **Detection density**: Radar vs LiDAR point count per frame at various ranges
- **Range accuracy**: Compare radar range estimates to LiDAR ground truth
- **Angular resolution**: Evaluate radar's ability to resolve closely-spaced objects
- **Material response**: Compare radar RCS returns from metal (cars), concrete (buildings), glass (windows), vegetation (trees)
- **Velocity estimation**: Validate Doppler velocity against ground-truth object velocities
- **Ghost detection analysis**: Identify and characterize multi-path artifacts from the WPM model
- **Occupancy grid comparison**: Build 2D occupancy grids from each sensor and compare coverage

---

## Phase 5 — Mapping & Fusion

### 5.1 LiDAR Mapping (Baseline)

- Use **SLAM Toolbox** or **Cartographer** via ROS2 to build a 2D/3D map from LiDAR
- Serves as the ground-truth reference map

### 5.2 Radar-Based Mapping

- Accumulate radar detections over time using odometry
- Filter by RCS threshold and SNR to remove noise
- Build radar occupancy grid or sparse 3D map
- Compare to LiDAR map: coverage gaps, false positives, material-dependent blind spots

### 5.3 Sensor Fusion

- Fuse radar + LiDAR point clouds using timestamp synchronization and TF alignment
- Explore EKF/UKF-based fusion for object tracking (radar provides velocity, LiDAR provides geometry)
- Evaluate fusion benefit vs single-sensor performance

---

## Phase 6 — Iteration & Expansion

### 6.1 Scenario Variations

- **Weather**: Add rain/fog particle effects to test radar robustness vs LiDAR degradation
- **Dynamic objects**: Add pedestrians, cyclists, moving vehicles with scripted or AI-driven behaviors
- **Time of day**: Test camera degradation at night while radar/LiDAR remain functional
- **Clutter**: Add dense vegetation, chain-link fences, metallic structures to stress radar multi-path

### 6.2 Radar Configuration Sweeps

- Vary radar parameters (range resolution, FOV, CFAR sensitivity) and measure impact on detection quality
- Compare `DmatApproxRadar` vs `WpmDmatApproxRadar` outputs systematically

### 6.3 Multiple Radar Configurations

- Mount multiple radars (front, rear, side-facing) for 360-degree coverage
- Evaluate interference and coverage overlap

---

## Milestone Tracker

| Milestone | Deliverable | Status |
|---|---|---|
| **M0** | Docker infrastructure operational (both containers start, GPU works) | ✅ COMPLETE |
| **M1** | Isaac Sim running with urban scene, robot spawned and controllable | ✅ COMPLETE — runtime verified |
| **M2** | Radar data visible in RViz2 | ✅ COMPLETE — 13–16 CFAR detections/frame at 97 Hz |
| **M3** | ROS2 bridge operational — radar & lidar point clouds visible in RViz2 | ✅ COMPLETE — both topics at 97 Hz, side-by-side in RViz2 |
| **M4** | Offline analysis scripts producing comparison plots | ✅ COMPLETE — 314s bag, 4 plots, fusion metrics CSV, analysis report |
| **M5** | Comprehensive analysis report — 10-section REPORT.md covering coverage, accuracy, RCS, occupancy map, fusion metrics | ✅ COMPLETE |
| **M6** | Material-aware radar tuning | `TODO` |
| **M7** | Final documentation and reproducibility | `TODO` |

---

## Key Risks & Mitigations

| Risk | Mitigation |
|---|---|
| No built-in city USD scene | Start with a small assembled town block (Option A); scale up incrementally |
| Radar GenericModelOutput format underdocumented | Inspect binary output with hex dump; reference `omni.sensors.nv.common` extension source |
| Radar has no native ROS2 publisher | Custom UDP → ROS2 bridge node (implemented in Phase 3.2) |
| GPU performance with large scene + multiple RTX sensors | Profile early; reduce scene complexity or sensor frame rates if needed |
| Material assignments affect radar fidelity | Audit all scene materials before running radar experiments |
| Isaac Sim 5.1 API differences from 4.5 | Review release notes; test scripts inside container before integration |
| Docker GPU passthrough issues | Validate NVIDIA Container Toolkit setup early; test with simple CUDA container first |

---

## Decision Log

All significant decisions are recorded here with date and rationale.

| Date | Decision | Rationale | Alternatives Considered |
|---|---|---|---|
| 2026-02-13 | **Urban environment: Option A (Assemble from Omniverse/third-party assets)** | Best balance of control, realism, and time-to-first-result. No external tooling required beyond Isaac Sim's asset converter. | Option B (procedural generation via Blender), Option C (NVIDIA Smart City Blueprint) — both preserved as alternatives |
| 2026-02-23 | **Docker multi-container deployment** | Reproducibility, portability to cloud compute, clean environment isolation. Avoids host driver version conflicts (Isaac Sim 5.x requires 580+ driver). | Native install via Omniverse Launcher (deprecated — driver conflicts) |
| 2026-02-23 | **Isaac Sim 5.1.0** (upgrade from 4.5.0) | Latest GA release, best sensor and extension support. Docker makes driver requirements a non-issue. | 4.5.0 (compatible with current host driver but older), 6.0 (early preview, not GA) |
| 2026-02-23 | **ROS2 Humble** (in containers, replaces Jazzy) | Native match for Ubuntu 22.04 in NVIDIA's container base image. LTS release. | Jazzy (requires Ubuntu 24.04, not supported in Isaac Sim containers) |
| 2026-02-27 | **Isaac Sim 5.1 extension name migration** | `omni.isaac.ros2_bridge` → `isaacsim.ros2.bridge`; `omni.isaac.sensor` → `isaacsim.sensors.rtx`; OmniGraph node type strings follow same pattern. `omni.sensors.nv.*` unchanged (Omniverse layer). OmniRadar prim replaces Camera-based radar prim in 5.x. | N/A — breaking change mandated by NVIDIA |
| 2026-03-05 | **RTX radar fix: Xvfb virtual framebuffer** | Headless `python.sh` launch path triggers carb plugin startup order bug in 5.1 — radar plugins crash before their required interfaces are registered. `xvfb-run -a` corrects initialization order. Chosen over downgrading to 4.5 (more rework) or using `isaac-sim.sh --exec` (less tested path). | Downgrade to 4.5; upgrade to 5.2+; use `isaac-sim.sh --exec` |
| 2026-03-30 | **Environment scaling: Video-to-3D reconstruction** | Photorealistic suburban environments from real-world video (KITTI-360, nuScenes, custom captures) via COLMAP + 3DGUT/Gaussian Splatting + material segmentation. Critical gap: automated material assignment for WpmDmatApproxRadar requires semantic segmentation pipeline (RMSNet/SAM+CLIP → mesh projection → Isaac Sim non-visual material bindings). To be pursued after M5–M7 complete. | Procedural generation (Blender + SceneCity — lower risk, materials correct by construction, fallback if video-to-3D material pipeline proves too costly), Hybrid procedural + SimReady assets, manual assembly from Omniverse assets |

---

## References

- [Isaac Sim 5.1 Documentation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/)
- [Isaac Sim 5.1 Container Installation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_container.html)
- [Isaac Sim 5.1 Radar Extension](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/omni_sensors_docs/radar_extension.html)
- [Isaac Sim 5.1 ROS2 Installation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html)
- [Isaac Sim NGC Container](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim)
- [Isaac Sim 5.1 Robot Assets](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/assets/usd_assets_robots.html)
- [Isaac Sim 5.1 RTX Lidar](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_physx_lidar.html)
- [RTX Lidar ROS2 Tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_rtx_lidar.html)
- [Custom Mobile Robot in Isaac Sim](https://manavthakkar.github.io/blog/taking-your-custom-mobile-robot-from-cad-to-isaac-sim)
- [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/)
