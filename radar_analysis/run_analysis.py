"""Offline analysis pipeline: reads a ROS2 bag and produces comparison metrics + plots.

Works without a live ROS2 installation — uses the `rosbags` pure-Python library.

Usage:
    python3 run_analysis.py <bag_path> [--output-dir analysis_output/] [--no-plots]

Example:
    python3 radar_analysis/run_analysis.py bags/run_20260225_143000 --output-dir analysis_output/
"""

import argparse
import os
import struct
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

# Allow running from project root or radar_analysis/ directory
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from compare_clouds import compare_summary
from visualize import (
    plot_bev,
    plot_comparison_metrics,
    plot_range_histogram,
    plot_rcs_vs_range,
)

# Radar PointCloud2 layout (matches radar_to_ros2.RADAR_FIELDS)
_RADAR_POINT_STEP = 24  # 6 × float32
_RADAR_OFFSETS = {"x": 0, "y": 4, "z": 8, "velocity": 12, "rcs": 16, "snr": 20}

# Topics to read
_RADAR_TOPIC = "/radar/point_cloud"
_LIDAR_TOPIC = "/lidar/point_cloud"


def _parse_radar_cloud(msg) -> dict:
    """Extract x,y,z,rcs,snr arrays from a radar PointCloud2 message."""
    n = msg.width * msg.height
    if n == 0:
        empty = np.empty(0, dtype=np.float32)
        return {"x": empty, "y": empty, "z": empty, "rcs": empty, "snr": empty}

    raw = bytes(msg.data)
    buf = np.frombuffer(raw, dtype=np.float32).reshape(n, _RADAR_POINT_STEP // 4)
    return {
        "x":   buf[:, _RADAR_OFFSETS["x"]   // 4],
        "y":   buf[:, _RADAR_OFFSETS["y"]   // 4],
        "z":   buf[:, _RADAR_OFFSETS["z"]   // 4],
        "rcs": buf[:, _RADAR_OFFSETS["rcs"] // 4],
        "snr": buf[:, _RADAR_OFFSETS["snr"] // 4],
    }


def _parse_lidar_cloud(msg) -> np.ndarray:
    """Extract xyz (N,3) from a LiDAR PointCloud2 message (x,y,z at offsets 0,4,8)."""
    n = msg.width * msg.height
    if n == 0:
        return np.empty((0, 3), dtype=np.float32)
    raw = bytes(msg.data)
    step = msg.point_step
    # Vectorised extraction of x, y, z using strides
    view = np.frombuffer(raw, dtype=np.uint8)
    xyz = np.empty((n, 3), dtype=np.float32)
    for col, offset in enumerate((0, 4, 8)):
        col_bytes = np.array([view[i * step + offset: i * step + offset + 4].tobytes()
                               for i in range(n)])
        xyz[:, col] = np.frombuffer(b"".join(col_bytes.tolist()), dtype=np.float32)
    return xyz


def read_bag(bag_path: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Read radar and LiDAR point clouds from a rosbag2 file.

    Returns:
        radar_xyz   — (N, 3) float32
        lidar_xyz   — (M, 3) float32
        radar_rcs   — (N,) float32
        lidar_ranges — (M,) float32 (Euclidean range from origin)
    """
    try:
        from rosbags.rosbag2 import Reader
        from rosbags.typesys import get_typestore, Stores
    except ImportError as e:
        print(f"ERROR: {e}")
        print("Install with: pip install rosbags>=0.9")
        sys.exit(1)

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    radar_parts: list[np.ndarray] = []
    radar_rcs_parts: list[np.ndarray] = []
    lidar_parts: list[np.ndarray] = []

    with Reader(bag_path) as reader:
        conns = [c for c in reader.connections
                 if c.topic in (_RADAR_TOPIC, _LIDAR_TOPIC)]
        if not conns:
            raise ValueError(f"No radar or lidar topics found in {bag_path}. "
                             f"Expected {_RADAR_TOPIC!r} and/or {_LIDAR_TOPIC!r}.")

        for connection, _timestamp, rawdata in reader.messages(connections=conns):
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            if connection.topic == _RADAR_TOPIC:
                fields = _parse_radar_cloud(msg)
                if fields["x"].size:
                    radar_parts.append(
                        np.column_stack([fields["x"], fields["y"], fields["z"]])
                    )
                    radar_rcs_parts.append(fields["rcs"])
            elif connection.topic == _LIDAR_TOPIC:
                xyz = _parse_lidar_cloud(msg)
                if xyz.shape[0]:
                    lidar_parts.append(xyz)

    radar_xyz = np.vstack(radar_parts) if radar_parts else np.empty((0, 3), dtype=np.float32)
    lidar_xyz = np.vstack(lidar_parts) if lidar_parts else np.empty((0, 3), dtype=np.float32)
    radar_rcs = np.concatenate(radar_rcs_parts) if radar_rcs_parts else np.empty(0, dtype=np.float32)
    lidar_ranges = np.linalg.norm(lidar_xyz, axis=1).astype(np.float32)

    return radar_xyz, lidar_xyz, radar_rcs, lidar_ranges


def run_analysis(bag_path: str, output_dir: str, no_plots: bool = False) -> dict:
    """Run the full analysis pipeline on a bag file.

    Returns the metrics dict from compare_summary.
    """
    import matplotlib
    if no_plots or not os.environ.get("DISPLAY"):
        matplotlib.use("Agg")  # headless — no GUI window

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    print(f"Reading bag: {bag_path}")
    radar_xyz, lidar_xyz, radar_rcs, lidar_ranges = read_bag(bag_path)

    print(f"  Radar frames accumulated: {len(radar_xyz)} total detections")
    print(f"  LiDAR frames accumulated: {len(lidar_xyz)} total points")

    if len(radar_xyz) == 0 or len(lidar_xyz) == 0:
        print("WARNING: one or both clouds are empty — skipping comparison.")
        return {}

    # Metrics
    metrics = compare_summary(radar_xyz, lidar_xyz)
    print(f"  Coverage ratio:  {metrics['coverage_ratio']:.1%}")
    print(f"  Mean distance:   {metrics['mean_distance_m']:.3f} m")
    print(f"  Radar count:     {metrics['radar_count']}")
    print(f"  LiDAR count:     {metrics['lidar_count']}")

    if not no_plots:
        import matplotlib.pyplot as plt

        # BEV plot
        bev_path = str(out / "bev.png")
        plot_bev(radar_xyz[:, :2], lidar_xyz[:, :2], save_path=bev_path)
        plt.close("all")

        # Range histogram
        radar_ranges = np.linalg.norm(radar_xyz, axis=1)
        hist_path = str(out / "range_histogram.png")
        plot_range_histogram(radar_ranges, lidar_ranges, save_path=hist_path)
        plt.close("all")

        # RCS vs range (radar only)
        if len(radar_rcs):
            rcs_path = str(out / "rcs_vs_range.png")
            plot_rcs_vs_range(radar_ranges, radar_rcs, save_path=rcs_path)
            plt.close("all")

        # Metrics bar chart
        metrics_path = str(out / "metrics.png")
        plot_comparison_metrics(metrics, save_path=metrics_path)
        plt.close("all")

    # Markdown report
    _write_report(out, bag_path, metrics, no_plots)

    print(f"\nResults written to: {output_dir}")
    return metrics


def _write_report(out: Path, bag_path: str, metrics: dict, no_plots: bool):
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    lines = [
        "# Radar vs LiDAR Analysis Report",
        f"",
        f"**Bag file:** `{bag_path}`  ",
        f"**Generated:** {ts}  ",
        f"",
        "## Metrics",
        "",
        "| Metric | Value |",
        "|--------|-------|",
        f"| Radar detections | {metrics.get('radar_count', 'N/A')} |",
        f"| LiDAR points | {metrics.get('lidar_count', 'N/A')} |",
        f"| Mean radar→LiDAR distance | {metrics.get('mean_distance_m', float('nan')):.3f} m |",
        f"| Median radar→LiDAR distance | {metrics.get('median_distance_m', float('nan')):.3f} m |",
        f"| Max radar→LiDAR distance | {metrics.get('max_distance_m', float('nan')):.3f} m |",
        f"| Coverage ratio (≤{metrics.get('threshold_m', 0.5):.1f} m) | "
        f"{metrics.get('coverage_ratio', float('nan')):.1%} |",
        "",
    ]
    if not no_plots:
        lines += [
            "## Plots",
            "",
            "![Bird's Eye View](bev.png)",
            "![Range Histogram](range_histogram.png)",
            "![RCS vs Range](rcs_vs_range.png)",
            "![Metrics Summary](metrics.png)",
            "",
        ]
    report_path = out / "analysis_report.md"
    report_path.write_text("\n".join(lines))


def main():
    parser = argparse.ArgumentParser(
        description="Offline radar vs LiDAR analysis from a ROS2 bag file."
    )
    parser.add_argument("bag_path", help="Path to the rosbag2 directory")
    parser.add_argument(
        "--output-dir", default="analysis_output",
        help="Directory for output plots and report (default: analysis_output/)"
    )
    parser.add_argument(
        "--no-plots", action="store_true",
        help="Skip plot generation (metrics and report only)"
    )
    args = parser.parse_args()
    run_analysis(args.bag_path, args.output_dir, args.no_plots)


if __name__ == "__main__":
    main()
