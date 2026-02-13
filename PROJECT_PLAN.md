# Isaac Sim mmWave Radar — Urban Mobile Robot Simulation

> **Status**: Active / Living Document
> **Created**: 2026-02-13
> **Last Updated**: 2026-02-13
> **Isaac Sim Version**: 4.5.0

---

## Document Conventions

- **[CHOSEN]** — The approach we are actively pursuing.
- **[ALTERNATIVE]** — A viable option we have documented but not selected. May revisit.
- **[DEPRECATED]** — An approach that was tried and abandoned, with rationale noted.
- Items marked with `TODO` require action.
- Decision history is preserved in the [Decision Log](#decision-log) at the bottom.

---

## Project Goal

Build a realistic city/town environment in NVIDIA Isaac Sim 4.5 to simulate a mobile robot platform equipped with a mmWave radar and complementary sensors. Traverse the environment, capture radar returns, display point clouds, and compare radar data against LiDAR point clouds/maps.

---

## Phase 0 — Prerequisites & Infrastructure

### 0.1 Hardware/Software Requirements

- **GPU**: NVIDIA RTX 3080+ (RTX 4090 or A6000 recommended for large urban scenes + multiple RTX sensors)
- **Isaac Sim 4.5** installed via Omniverse Launcher
- **ROS2 Humble or Jazzy** — sensor data transport, visualization, and analysis
- **Isaac Sim ROS2 Bridge extension** enabled
- **Python 3.10+** with NumPy, Open3D, matplotlib for offline analysis

### 0.2 Enable Required Extensions

In Isaac Sim Extension Manager, enable:

- `omni.sensors.nv.radar` — [Radar Extension](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/sensors/omni_sensors_docs/radar_extension.html)
- `omni.sensors.nv.common` — GenericModelOutput format dependency
- `omni.isaac.ros2_bridge` — ROS2 integration
- `omni.isaac.sensor` — PhysX-based sensors (IMU, contact)
- RTX Lidar extension — comparison data

---

## Phase 1 — Urban Environment Creation

Isaac Sim 4.5 ships with **no built-in city/urban outdoor environments**. The bundled environments are indoor only (warehouse, hospital, office, simple rooms). We must build or source an urban scene.

### 1.1 Option A — Assemble from Omniverse/Third-Party Assets **[CHOSEN]**

> **Decision**: Selected as the starting approach on 2026-02-13. Offers the best balance of control, realism, and time-to-first-result without requiring procedural generation tooling or external streaming infrastructure.

- Use **NVIDIA Omniverse SimReady assets** (buildings, roads, vehicles, street furniture) from the Omniverse Asset Store
- Import city-block assets from **TurboSquid**, **Sketchfab**, or **CGTrader** in FBX/OBJ format
- Convert to USD using Isaac Sim's built-in **Asset Converter** (File > Import)
- Compose a town-scale scene (target: ~500m x 500m block with roads, intersections, buildings, parked cars, street signs, vegetation)

**Steps**:
1. `TODO` Survey available SimReady urban assets in the Omniverse launcher
2. `TODO` Identify and acquire supplemental third-party assets (roads, buildings, vehicles, vegetation)
3. `TODO` Convert all assets to USD; verify units (meters) and orientation (Z-up)
4. `TODO` Assemble a first draft town block in Isaac Sim Composer
5. `TODO` Assign physically-based materials to all surfaces (critical for radar — see Phase 2.3)
6. `TODO` Enable collisions on all geometry
7. `TODO` Add HDR sky dome lighting for outdoor rendering
8. `TODO` Generate NavMesh for autonomous navigation

### 1.2 Option B — Procedural City Generation **[ALTERNATIVE]**

> May revisit if Option A proves too labour-intensive or if we need larger/more varied environments.

- Use **Blender + add-ons** (e.g., SceneCity, Buildify) to procedurally generate a town layout, export as USD/FBX
- Alternatively, use **CesiumGS/cesium-omniverse** to stream real-world 3D tiles into Omniverse for photorealistic geo-referenced scenes
- Pros: Scales to large areas quickly, reproducible layouts
- Cons: Requires Blender pipeline expertise; Cesium streaming adds infrastructure complexity

### 1.3 Option C — NVIDIA Metropolis / Digital Twin Blueprint **[ALTERNATIVE]**

> The Smart City AI Blueprint targets infrastructure monitoring, not robotics simulation. Worth tracking as the ecosystem matures.

- The [NVIDIA Omniverse Blueprint for Smart Cities](https://blogs.nvidia.com/blog/smart-city-ai-blueprint-europe/) provides a pipeline for building city-scale digital twins
- Integrates Omniverse, Cosmos, NeMo, and Metropolis
- Pros: Photorealistic, city-scale, maintained by NVIDIA
- Cons: Not designed for robotics sim; may require significant adaptation

### 1.4 Environment Setup Checklist (applies to all options)

- [ ] Stage set to **meters** and **Z-up**
- [ ] **Collisions enabled** on all geometry (required for PhysX Lidar and physics)
- [ ] **Physically-based materials** assigned — `WpmDmatApproxRadar` uses material-dependent radiometry, so correct materials (metal, concrete, glass, vegetation) directly affect radar fidelity
- [ ] Appropriate **lighting** (HDR sky dome for outdoor)
- [ ] **NavMesh** generated (if using autonomous navigation)

---

## Phase 2 — Mobile Robot Platform Setup

### 2.1 Select or Build a Robot

- **Bundled option**: [Idealworks iw.hub](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/assets/usd_assets_robots.html) (`Robots/Idealworks/iw_hub_sensors.usd`) — mobile base with LiDAR and cameras pre-configured
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

### 3.1 Radar Data Pipeline

The radar extension outputs via the **TranscoderRadar** OmniGraph node (`omni.sensors.nv.radar.TranscoderRadar`), encoding detections into UDP packets in GenericModelOutput format:

```
RTX Radar Sensor → GPU Compute → TranscoderRadar → UDP (port 10001)
```

- Configure `remoteIP`, `remotePort`, `interfaceIP` on the TranscoderRadar node
- Optionally write to binary file via the `fileName` attribute for offline analysis

### 3.2 Radar-to-ROS2 Bridge (Custom — Required)

The radar extension does **not** natively publish to ROS2 topics. A bridge node is required:

**Approach 1 — UDP listener node**:
1. Write a ROS2 node (Python or C++) that listens on the UDP socket (default `239.0.0.1:10001`)
2. Parse the GenericModelOutput binary format (ref: `omni.sensors.nv.common` docs)
3. Publish as `sensor_msgs/PointCloud2` (fields: x, y, z, velocity, RCS, SNR)
4. Optionally publish a custom `RadarDetection` message for per-detection metadata

**Approach 2 — Isaac Sim Python scripting**:
- Read radar output buffers directly via the Isaac Sim Python API and publish via `rclpy`

### 3.3 LiDAR Data Pipeline (Native ROS2 Support)

LiDAR has native ROS2 support in Isaac Sim:
- Use **Tools > Robotics > ROS2 OmniGraphs > RTX LiDAR** for automatic publishing
- Publishes `sensor_msgs/PointCloud2` on `/point_cloud`
- Publishes `sensor_msgs/LaserScan` if configured

### 3.4 Frame / TF Configuration

- Publish TF tree: `map → odom → base_link → radar_link`, `base_link → lidar_link`, etc.
- Use **Tools > Robotics > ROS2 OmniGraphs > TF Publisher**
- Ensure radar and lidar frames are correctly offset to match physical mount positions

---

## Phase 4 — Visualization & Analysis

### 4.1 Real-Time Visualization (RViz2)

- **Radar point cloud**: Subscribe to radar PointCloud2 topic; color by RCS or velocity
- **LiDAR point cloud**: Subscribe to lidar PointCloud2 topic
- **Overlay both** in the same RViz2 session with different color maps for direct comparison
- Add robot model (URDF) visualization, TF frames, camera image panels

### 4.2 Isaac Sim Viewport Visualization

- Enable RTX Radar point cloud rendering in viewport (via OmniGraph if supported)
- Enable RTX LiDAR viewport rendering for side-by-side comparison within the simulator

### 4.3 Offline Analysis Pipeline

```
radar_analysis/
├── record_bags.py            # rosbag2 recording of all sensor topics
├── parse_radar_udp.py        # Parse raw GenericModelOutput binary files
├── radar_pointcloud.py       # Convert radar detections → Open3D point clouds
├── lidar_pointcloud.py       # Convert lidar scans → Open3D point clouds
├── compare_clouds.py         # Registration, ICP alignment, density comparison
├── radar_characteristics.py  # RCS analysis, detection probability vs range, velocity histograms
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
| **M1** | Isaac Sim running with urban scene, robot spawned and controllable | `TODO` |
| **M2** | Radar + LiDAR sensors mounted and producing data | `TODO` |
| **M3** | ROS2 bridge operational — radar & lidar point clouds visible in RViz2 | `TODO` |
| **M4** | Offline analysis scripts producing comparison plots | `TODO` |
| **M5** | LiDAR SLAM map generated, radar map generated, comparison complete | `TODO` |
| **M6** | Sensor fusion pipeline operational | `TODO` |
| **M7** | Scenario variations tested, results documented | `TODO` |

---

## Key Risks & Mitigations

| Risk | Mitigation |
|---|---|
| No built-in city USD scene | Start with a small assembled town block (Option A); scale up incrementally |
| Radar GenericModelOutput format underdocumented | Inspect binary output with hex dump; reference `omni.sensors.nv.common` extension source |
| Radar has no native ROS2 publisher | Build a custom UDP → ROS2 bridge node (Phase 3.2) |
| GPU performance with large scene + multiple RTX sensors | Profile early; reduce scene complexity or sensor frame rates if needed |
| Material assignments affect radar fidelity | Audit all scene materials before running radar experiments |

---

## Decision Log

All significant decisions are recorded here with date and rationale.

| Date | Decision | Rationale | Alternatives Considered |
|---|---|---|---|
| 2026-02-13 | **Urban environment: Option A (Assemble from Omniverse/third-party assets)** | Best balance of control, realism, and time-to-first-result. No external tooling required beyond Isaac Sim's asset converter. | Option B (procedural generation via Blender), Option C (NVIDIA Smart City Blueprint) — both preserved as alternatives |

---

## References

- [Isaac Sim 4.5 Radar Extension](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/sensors/omni_sensors_docs/radar_extension.html)
- [Isaac Sim 4.5 Environment Assets](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/assets/usd_assets_environments.html)
- [Isaac Sim 4.5 Robot Assets](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/assets/usd_assets_robots.html)
- [Isaac Sim 4.5 PhysX Lidar](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/sensors/isaacsim_sensors_physx_lidar.html)
- [RTX Lidar ROS2 Tutorial](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/ros2_tutorials/tutorial_ros2_rtx_lidar.html)
- [Custom Mobile Robot in Isaac Sim](https://manavthakkar.github.io/blog/taking-your-custom-mobile-robot-from-cad-to-isaac-sim)
- [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/)
- [NVIDIA Smart City AI Blueprint](https://blogs.nvidia.com/blog/smart-city-ai-blueprint-europe/)
