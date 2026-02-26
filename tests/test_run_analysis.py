"""Tests for the offline analysis pipeline (run_analysis.py + analysis modules).

Uses synthetic point clouds — no bag files, no ROS2, no Isaac Sim required.
Forces matplotlib into non-interactive Agg backend so plots save without a display.
"""

import os
import sys
import tempfile
import unittest

import matplotlib
matplotlib.use("Agg")  # must be set before any other matplotlib import
import matplotlib.pyplot as plt

import numpy as np

# Allow running from project root
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "radar_analysis"))

from compare_clouds import compare_summary
from visualize import (
    plot_bev,
    plot_comparison_metrics,
    plot_range_histogram,
    plot_rcs_vs_range,
)


def _make_radar_cloud(n: int = 50, seed: int = 42) -> np.ndarray:
    """Generate a synthetic radar point cloud (N, 3) at short-to-medium range."""
    rng = np.random.default_rng(seed)
    angles = rng.uniform(-np.pi / 4, np.pi / 4, n)
    ranges = rng.uniform(5.0, 40.0, n)
    x = (ranges * np.cos(angles)).astype(np.float32)
    y = (ranges * np.sin(angles)).astype(np.float32)
    z = rng.uniform(-0.5, 0.5, n).astype(np.float32)
    return np.column_stack([x, y, z])


def _make_lidar_cloud(n: int = 500, seed: int = 7) -> np.ndarray:
    """Generate a denser synthetic LiDAR cloud (N, 3) covering a wider area."""
    rng = np.random.default_rng(seed)
    angles = rng.uniform(-np.pi / 2, np.pi / 2, n)
    ranges = rng.uniform(1.0, 60.0, n)
    x = (ranges * np.cos(angles)).astype(np.float32)
    y = (ranges * np.sin(angles)).astype(np.float32)
    z = rng.uniform(-1.0, 1.0, n).astype(np.float32)
    return np.column_stack([x, y, z])


class TestCompareSummary(unittest.TestCase):
    def setUp(self):
        self.radar = _make_radar_cloud()
        self.lidar = _make_lidar_cloud()

    def test_summary_has_all_keys(self):
        result = compare_summary(self.radar, self.lidar)
        expected_keys = {
            "radar_count", "lidar_count",
            "mean_distance_m", "median_distance_m", "max_distance_m",
            "coverage_ratio", "threshold_m",
        }
        self.assertEqual(set(result.keys()), expected_keys)

    def test_counts_match_inputs(self):
        result = compare_summary(self.radar, self.lidar)
        self.assertEqual(result["radar_count"], len(self.radar))
        self.assertEqual(result["lidar_count"], len(self.lidar))

    def test_distances_are_non_negative(self):
        result = compare_summary(self.radar, self.lidar)
        self.assertGreaterEqual(result["mean_distance_m"], 0.0)
        self.assertGreaterEqual(result["median_distance_m"], 0.0)
        self.assertGreaterEqual(result["max_distance_m"], 0.0)

    def test_coverage_ratio_in_range(self):
        result = compare_summary(self.radar, self.lidar)
        self.assertGreaterEqual(result["coverage_ratio"], 0.0)
        self.assertLessEqual(result["coverage_ratio"], 1.0)

    def test_identical_clouds_zero_distance(self):
        result = compare_summary(self.radar, self.radar)
        self.assertAlmostEqual(result["mean_distance_m"], 0.0, places=5)
        self.assertAlmostEqual(result["coverage_ratio"], 1.0, places=5)


class TestVisualizeSavesFiles(unittest.TestCase):
    def setUp(self):
        self.radar = _make_radar_cloud()
        self.lidar = _make_lidar_cloud()
        self.metrics = compare_summary(self.radar, self.lidar)
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        plt.close("all")

    def test_plot_bev_saves_png(self):
        path = os.path.join(self.tmp, "bev.png")
        plot_bev(self.radar[:, :2], self.lidar[:, :2], save_path=path)
        plt.close("all")
        self.assertTrue(os.path.exists(path))
        self.assertGreater(os.path.getsize(path), 0)

    def test_plot_range_histogram_saves_png(self):
        radar_ranges = np.linalg.norm(self.radar, axis=1)
        lidar_ranges = np.linalg.norm(self.lidar, axis=1)
        path = os.path.join(self.tmp, "hist.png")
        plot_range_histogram(radar_ranges, lidar_ranges, save_path=path)
        plt.close("all")
        self.assertTrue(os.path.exists(path))
        self.assertGreater(os.path.getsize(path), 0)

    def test_plot_rcs_vs_range_saves_png(self):
        rng = np.random.default_rng(1)
        ranges = np.linalg.norm(self.radar, axis=1)
        rcs = rng.uniform(-15, 5, len(ranges)).astype(np.float32)
        path = os.path.join(self.tmp, "rcs.png")
        plot_rcs_vs_range(ranges, rcs, save_path=path)
        plt.close("all")
        self.assertTrue(os.path.exists(path))
        self.assertGreater(os.path.getsize(path), 0)

    def test_plot_comparison_metrics_saves_png(self):
        path = os.path.join(self.tmp, "metrics.png")
        plot_comparison_metrics(self.metrics, save_path=path)
        plt.close("all")
        self.assertTrue(os.path.exists(path))
        self.assertGreater(os.path.getsize(path), 0)


if __name__ == "__main__":
    unittest.main()
