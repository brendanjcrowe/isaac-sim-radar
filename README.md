# isaac-sim-radar

mmWave radar simulation on a mobile robot platform in NVIDIA Isaac Sim 4.5, with LiDAR comparison and sensor fusion.

## Overview

This project builds a realistic urban environment in Isaac Sim to simulate a mobile robot equipped with:

- **mmWave Radar** (RTX Radar — `WpmDmatApproxRadar`) as the primary sensor under study
- **LiDAR** (RTX Lidar) for ground-truth comparison
- **RGB Camera**, **IMU**, and optional **GPS** for full perception context

The goal is to traverse the environment, capture and analyse radar returns, visualise point clouds, and compare radar performance against LiDAR maps.

## Project Structure

```
isaac-sim-radar/
├── PROJECT_PLAN.md          # Living project plan with decision log
├── README.md
├── LICENSE                  # Apache 2.0
└── .gitignore
```

## Key Documentation

- **[PROJECT_PLAN.md](PROJECT_PLAN.md)** — Full project plan covering environment creation, robot/sensor setup, data pipeline, analysis, and fusion. This is a living document that tracks decisions, alternatives considered, and current status.

## Requirements

- NVIDIA GPU: RTX 3080+ (RTX 4090 / A6000 recommended)
- [NVIDIA Isaac Sim 4.5](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/)
- ROS2 Humble or Jazzy
- Python 3.10+ (NumPy, Open3D, matplotlib)

## Isaac Sim Extensions Used

| Extension | Purpose |
|---|---|
| `omni.sensors.nv.radar` | [RTX Radar sensor](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/sensors/omni_sensors_docs/radar_extension.html) |
| `omni.sensors.nv.common` | GenericModelOutput format |
| `omni.isaac.ros2_bridge` | ROS2 integration |
| `omni.isaac.sensor` | PhysX sensors (IMU, contact) |
| RTX Lidar extension | LiDAR comparison data |

## Current Status

See the [Milestone Tracker in PROJECT_PLAN.md](PROJECT_PLAN.md#milestone-tracker) for detailed progress.

## License

Apache 2.0 — see [LICENSE](LICENSE).
