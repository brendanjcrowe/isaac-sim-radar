"""Tests for radar analysis modules (no ROS2 required)."""

import unittest
import numpy as np
import sys, os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "radar_analysis"))

from compare_clouds import compute_distances, coverage_ratio, compare_summary
from radar_pointcloud import detections_to_pointcloud
from lidar_pointcloud import pointcloud2_to_numpy


class TestCompareDistances(unittest.TestCase):

    def test_identical_points(self):
        pts = np.array([[0, 0, 0], [1, 1, 1]], dtype=np.float64)
        dists = compute_distances(pts, pts)
        np.testing.assert_allclose(dists, [0.0, 0.0], atol=1e-6)

    def test_known_distance(self):
        source = np.array([[0, 0, 0]], dtype=np.float64)
        target = np.array([[3, 4, 0]], dtype=np.float64)
        dists = compute_distances(source, target)
        np.testing.assert_allclose(dists, [5.0], atol=1e-6)

    def test_nearest_neighbor(self):
        source = np.array([[0, 0, 0]], dtype=np.float64)
        target = np.array([[1, 0, 0], [10, 0, 0]], dtype=np.float64)
        dists = compute_distances(source, target)
        np.testing.assert_allclose(dists, [1.0], atol=1e-6)


class TestCoverageRatio(unittest.TestCase):

    def test_full_coverage(self):
        pts = np.random.randn(50, 3)
        ratio = coverage_ratio(pts, pts, threshold_m=0.01)
        self.assertAlmostEqual(ratio, 1.0)

    def test_no_coverage(self):
        radar = np.array([[0, 0, 0]], dtype=np.float64)
        lidar = np.array([[100, 100, 100]], dtype=np.float64)
        ratio = coverage_ratio(radar, lidar, threshold_m=0.5)
        self.assertAlmostEqual(ratio, 0.0)

    def test_empty_radar(self):
        radar = np.empty((0, 3))
        lidar = np.array([[1, 2, 3]], dtype=np.float64)
        ratio = coverage_ratio(radar, lidar)
        self.assertEqual(ratio, 0.0)


class TestCompareSummary(unittest.TestCase):

    def test_summary_keys(self):
        radar = np.random.randn(20, 3)
        lidar = np.random.randn(100, 3)
        result = compare_summary(radar, lidar)

        expected_keys = {"radar_count", "lidar_count", "mean_distance_m",
                         "median_distance_m", "max_distance_m",
                         "coverage_ratio", "threshold_m"}
        self.assertEqual(set(result.keys()), expected_keys)
        self.assertEqual(result["radar_count"], 20)
        self.assertEqual(result["lidar_count"], 100)


class TestRadarPointcloud(unittest.TestCase):

    def test_create_pointcloud(self):
        x = np.array([1.0, 2.0, 3.0], dtype=np.float32)
        y = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        z = np.array([0.5, 0.5, 0.5], dtype=np.float32)

        pcd = detections_to_pointcloud(x, y, z)
        self.assertEqual(len(pcd.points), 3)

    def test_create_pointcloud_with_color(self):
        n = 10
        x = np.random.randn(n).astype(np.float32)
        y = np.random.randn(n).astype(np.float32)
        z = np.random.randn(n).astype(np.float32)
        rcs = np.random.randn(n).astype(np.float32)

        pcd = detections_to_pointcloud(x, y, z, color_by=rcs)
        self.assertEqual(len(pcd.points), n)
        self.assertEqual(len(pcd.colors), n)


if __name__ == "__main__":
    unittest.main()
