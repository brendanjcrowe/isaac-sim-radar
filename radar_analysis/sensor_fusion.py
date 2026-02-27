"""Offline radar + LiDAR sensor fusion from a recorded rosbag2.

Fusion approach
---------------
1. Synchronise radar and LiDAR PointCloud2 messages by nearest timestamp
   (within a configurable max gap).
2. For each synchronised pair compute:
     - detection_ratio        : n_radar / n_lidar
     - coverage_complement_m  : mean distance from each radar point to
                                its nearest LiDAR point (2D, XY plane)
     - coverage_pct_covered   : fraction of radar points within 0.5 m of
                                at least one LiDAR point
     - mean_radar_velocity    : mean |Doppler velocity| across radar pts
3. Aggregate per-frame metrics into a CSV file.
4. Save a 3-panel summary plot.

Usage (standalone):
    python3 radar_analysis/sensor_fusion.py bags/session_001 --out analysis_output
"""

import argparse
import csv
import os
import struct
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class FusionFrame:
    """Metrics for one synchronised (radar, lidar) frame pair."""
    timestamp_ns:         int
    n_radar:              int
    n_lidar:              int
    detection_ratio:      float   # n_radar / n_lidar  (nan if lidar == 0)
    coverage_complement_m: float  # mean dist from radar pt to nearest lidar pt
    coverage_pct_covered: float   # fraction of radar pts within coverage_threshold
    mean_radar_velocity:  float   # mean |Doppler velocity| (m/s); nan if unavailable


def fuse_pair(
    radar_xyz: np.ndarray,
    radar_vel: Optional[np.ndarray],
    lidar_xyz: np.ndarray,
    coverage_threshold: float = 0.5,
) -> FusionFrame:
    """Compute fusion metrics for one synchronised (radar, lidar) pair.

    Args:
        radar_xyz: (N, 3) float32 array of radar detection positions.
        radar_vel: (N,)  float32 Doppler velocities, or None.
        lidar_xyz: (M, 3) float32 array of LiDAR point positions.
        coverage_threshold: distance (m) below which a radar point is
            considered "covered" by a LiDAR point.

    Returns:
        FusionFrame with per-frame metrics (timestamp_ns left at 0;
        caller should set it after the fact).
    """
    n_radar = len(radar_xyz)
    n_lidar = len(lidar_xyz)

    ratio = float(n_radar) / float(n_lidar) if n_lidar > 0 else float("nan")

    if n_radar == 0 or n_lidar == 0:
        complement_m = float("nan")
        pct_covered  = float("nan")
    else:
        # 2D nearest-neighbour (XY plane only — both sensors share base_link Z)
        r2d = radar_xyz[:, :2]
        l2d = lidar_xyz[:, :2]
        # Vectorised: (N, M, 2) → (N, M) → (N,)
        diffs     = r2d[:, np.newaxis, :] - l2d[np.newaxis, :, :]
        dists     = np.sqrt((diffs ** 2).sum(axis=2))
        min_dists = dists.min(axis=1)
        complement_m = float(min_dists.mean())
        pct_covered  = float((min_dists < coverage_threshold).mean())

    mean_vel = float(np.abs(radar_vel).mean()) if (radar_vel is not None and n_radar > 0) else float("nan")

    return FusionFrame(
        timestamp_ns=0,
        n_radar=n_radar,
        n_lidar=n_lidar,
        detection_ratio=ratio,
        coverage_complement_m=complement_m,
        coverage_pct_covered=pct_covered,
        mean_radar_velocity=mean_vel,
    )


def save_metrics_csv(frames: List[FusionFrame], path: str) -> None:
    """Write per-frame fusion metrics to a CSV file."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fieldnames = [
        "timestamp_ns", "n_radar", "n_lidar", "detection_ratio",
        "coverage_complement_m", "coverage_pct_covered", "mean_radar_velocity",
    ]
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for fr in frames:
            writer.writerow({
                "timestamp_ns":            fr.timestamp_ns,
                "n_radar":                 fr.n_radar,
                "n_lidar":                 fr.n_lidar,
                "detection_ratio":         _fmt(fr.detection_ratio),
                "coverage_complement_m":   _fmt(fr.coverage_complement_m),
                "coverage_pct_covered":    _fmt(fr.coverage_pct_covered),
                "mean_radar_velocity":     _fmt(fr.mean_radar_velocity),
            })


def _fmt(v: float) -> str:
    return "nan" if (v != v) else f"{v:.4f}"   # nan != nan


def plot_fusion_summary(frames: List[FusionFrame], out_dir: str) -> str:
    """Save a 3-panel summary plot of per-frame fusion metrics."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ts = np.array([f.timestamp_ns / 1e9 for f in frames])
    if len(ts):
        ts -= ts[0]

    ratios  = np.array([f.detection_ratio        for f in frames])
    compl   = np.array([f.coverage_complement_m   for f in frames])
    covered = np.array([f.coverage_pct_covered * 100 for f in frames])

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    axes[0].plot(ts, ratios, color="tab:orange")
    axes[0].set_ylabel("Radar / LiDAR point ratio")
    axes[0].set_title("Sensor Fusion — Per-Frame Metrics")
    axes[0].axhline(1.0, ls="--", color="grey", lw=0.8)

    axes[1].plot(ts, compl, color="tab:blue")
    axes[1].set_ylabel("Mean dist to nearest LiDAR pt (m)")

    axes[2].plot(ts, covered, color="tab:green")
    axes[2].set_ylabel("% radar pts within 0.5 m of LiDAR")
    axes[2].set_ylim(0, 105)
    axes[2].set_xlabel("Time (s)")

    fig.tight_layout()
    path = os.path.join(out_dir, "fusion_summary.png")
    os.makedirs(out_dir, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


# ── Bag-level driver ──────────────────────────────────────────────────────────

def _unpack_xyz_vel(msg) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Extract XYZ and optional velocity from a sensor_msgs/PointCloud2."""
    fields_map = {f.name: f for f in msg.fields}
    ps   = msg.point_step
    data = bytes(msg.data)
    n    = msg.width * msg.height

    def _col(name: str) -> Optional[np.ndarray]:
        if name not in fields_map:
            return None
        off = fields_map[name].offset
        return np.array(
            [struct.unpack_from("f", data, i * ps + off)[0] for i in range(n)],
            dtype=np.float32,
        )

    x   = _col("x")
    y   = _col("y")
    z   = _col("z")
    vel = _col("velocity")

    if x is None: x = np.zeros(n, np.float32)
    if y is None: y = np.zeros(n, np.float32)
    if z is None: z = np.zeros(n, np.float32)

    xyz = np.column_stack([x, y, z]) if n > 0 else np.zeros((0, 3), np.float32)
    return xyz, vel


def run_from_bag(
    bag_path: str,
    out_dir: str,
    radar_topic: str = "/radar/point_cloud",
    lidar_topic: str = "/lidar/point_cloud",
    max_dt_ns: int = 100_000_000,   # 100 ms — max acceptable sync gap
) -> List[FusionFrame]:
    """Read a rosbag2 and produce per-frame fusion metrics.

    Writes fusion_metrics.csv and fusion_summary.png to out_dir.
    Returns the list of FusionFrame objects for downstream use.
    """
    from rosbags.rosbag2 import Reader
    from rosbags.serde import deserialize_cdr

    radar_msgs: List[Tuple[int, bytes, str]] = []
    lidar_msgs: List[Tuple[int, bytes, str]] = []

    with Reader(bag_path) as reader:
        for conn, stamp, rawdata in reader.messages():
            if conn.topic == radar_topic:
                radar_msgs.append((stamp, rawdata, conn.msgtype))
            elif conn.topic == lidar_topic:
                lidar_msgs.append((stamp, rawdata, conn.msgtype))

    if not radar_msgs or not lidar_msgs:
        print(f"[sensor_fusion] No messages found on one or both topics.")
        return []

    lidar_stamps = np.array([t for t, _, _ in lidar_msgs], dtype=np.int64)
    frames: List[FusionFrame] = []

    for r_stamp, r_raw, r_type in radar_msgs:
        idx     = int(np.argmin(np.abs(lidar_stamps - r_stamp)))
        l_stamp, l_raw, l_type = lidar_msgs[idx]

        if abs(int(l_stamp) - int(r_stamp)) > max_dt_ns:
            continue   # too far apart — skip

        r_msg = deserialize_cdr(r_raw, r_type)
        l_msg = deserialize_cdr(l_raw, l_type)

        radar_xyz, radar_vel = _unpack_xyz_vel(r_msg)
        lidar_xyz, _         = _unpack_xyz_vel(l_msg)

        frame = fuse_pair(radar_xyz, radar_vel, lidar_xyz)
        frame.timestamp_ns = r_stamp
        frames.append(frame)

    os.makedirs(out_dir, exist_ok=True)
    if frames:
        save_metrics_csv(frames, os.path.join(out_dir, "fusion_metrics.csv"))
        plot_fusion_summary(frames, out_dir)
        print(f"[sensor_fusion] {len(frames)} frame pairs fused → {out_dir}/")

    return frames


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Fuse radar + LiDAR from a rosbag2.")
    ap.add_argument("bag",           help="Path to rosbag2 directory")
    ap.add_argument("--out",         default="analysis_output")
    ap.add_argument("--radar-topic", default="/radar/point_cloud")
    ap.add_argument("--lidar-topic", default="/lidar/point_cloud")
    ap.add_argument("--max-dt-ms",   type=int, default=100,
                    help="Max sync gap in milliseconds (default 100).")
    args = ap.parse_args()

    frames = run_from_bag(
        args.bag, args.out,
        args.radar_topic, args.lidar_topic,
        max_dt_ns=args.max_dt_ms * 1_000_000,
    )
    if frames:
        ratios = [f.detection_ratio for f in frames if not (f.detection_ratio != f.detection_ratio)]
        print(f"Mean radar/lidar ratio : {np.mean(ratios):.3f}" if ratios else "No valid ratios.")
    else:
        print("No synchronised pairs found.")
