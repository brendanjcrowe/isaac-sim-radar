"""Visualization utilities for radar vs LiDAR comparison."""

import numpy as np
import matplotlib.pyplot as plt


def plot_bev(radar_xy: np.ndarray, lidar_xy: np.ndarray = None,
             title: str = "Bird's Eye View", save_path: str = None):
    """Plot bird's-eye view (X-Y plane) of radar and optionally LiDAR."""
    fig, ax = plt.subplots(1, 1, figsize=(10, 10))

    if lidar_xy is not None and len(lidar_xy) > 0:
        ax.scatter(lidar_xy[:, 0], lidar_xy[:, 1], s=1, c="green",
                   alpha=0.3, label=f"LiDAR ({len(lidar_xy)})")

    if len(radar_xy) > 0:
        ax.scatter(radar_xy[:, 0], radar_xy[:, 1], s=10, c="red",
                   alpha=0.8, label=f"Radar ({len(radar_xy)})")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(title)
    ax.set_aspect("equal")
    ax.legend()
    ax.grid(True, alpha=0.3)

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.show()


def plot_range_histogram(radar_ranges: np.ndarray, lidar_ranges: np.ndarray = None,
                         bins: int = 50, save_path: str = None):
    """Plot histogram of detection ranges."""
    fig, ax = plt.subplots(1, 1, figsize=(10, 5))

    ax.hist(radar_ranges, bins=bins, alpha=0.7, color="red", label="Radar")
    if lidar_ranges is not None:
        ax.hist(lidar_ranges, bins=bins, alpha=0.5, color="green", label="LiDAR")

    ax.set_xlabel("Range (m)")
    ax.set_ylabel("Count")
    ax.set_title("Detection Range Distribution")
    ax.legend()

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.show()


def plot_rcs_vs_range(ranges: np.ndarray, rcs: np.ndarray, save_path: str = None):
    """Scatter plot of RCS vs range for radar detections."""
    fig, ax = plt.subplots(1, 1, figsize=(10, 5))

    scatter = ax.scatter(ranges, rcs, s=5, c=rcs, cmap="hot", alpha=0.7)
    fig.colorbar(scatter, ax=ax, label="RCS (dBsm)")

    ax.set_xlabel("Range (m)")
    ax.set_ylabel("RCS (dBsm)")
    ax.set_title("Radar Cross Section vs Range")

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.show()


def plot_comparison_metrics(metrics: dict, save_path: str = None):
    """Bar chart of comparison metrics from compare_clouds.compare_summary()."""
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # Point counts
    axes[0].bar(["Radar", "LiDAR"],
                [metrics["radar_count"], metrics["lidar_count"]],
                color=["red", "green"])
    axes[0].set_title("Detection Count")
    axes[0].set_ylabel("Points")

    # Distance metrics
    dist_names = ["Mean", "Median", "Max"]
    dist_values = [metrics["mean_distance_m"], metrics["median_distance_m"],
                   metrics["max_distance_m"]]
    axes[1].bar(dist_names, dist_values, color="steelblue")
    axes[1].set_title(f"Radar→LiDAR Distance (coverage={metrics['coverage_ratio']:.1%})")
    axes[1].set_ylabel("Distance (m)")

    fig.suptitle("Radar vs LiDAR Comparison")
    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.show()
