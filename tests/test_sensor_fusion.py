"""Tests for radar_analysis/sensor_fusion.py."""

import csv
import math
import os
import sys
import tempfile

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from radar_analysis.sensor_fusion import (
    FusionFrame,
    fuse_pair,
    plot_fusion_summary,
    save_metrics_csv,
)


def _xyz(pts):
    """Helper: list of (x,y,z) → float32 ndarray."""
    return np.array(pts, dtype=np.float32)


class TestFusePair:
    def test_identical_clouds_fully_covered(self):
        pts = _xyz([(1, 0, 0), (2, 0, 0), (3, 0, 0)])
        frame = fuse_pair(pts, None, pts)
        assert frame.coverage_pct_covered == pytest.approx(1.0)
        assert frame.coverage_complement_m == pytest.approx(0.0, abs=1e-5)

    def test_disjoint_clouds_not_covered(self):
        radar = _xyz([(10, 0, 0), (20, 0, 0)])
        lidar = _xyz([(0, 0, 0), (0, 1, 0)])
        frame = fuse_pair(radar, None, lidar, coverage_threshold=0.5)
        assert frame.coverage_pct_covered == pytest.approx(0.0)

    def test_detection_ratio(self):
        radar = _xyz([(1, 0, 0), (2, 0, 0)])   # 2 points
        lidar = _xyz([(1, 0, 0)] * 10)          # 10 points
        frame = fuse_pair(radar, None, lidar)
        assert frame.detection_ratio == pytest.approx(2 / 10)

    def test_empty_radar(self):
        lidar = _xyz([(1, 0, 0), (2, 0, 0)])
        frame = fuse_pair(np.zeros((0, 3), np.float32), None, lidar)
        assert frame.n_radar == 0
        assert math.isnan(frame.coverage_complement_m)
        assert math.isnan(frame.coverage_pct_covered)
        # 0 radar / 2 lidar = 0.0 (not nan — zero is a valid, meaningful ratio)
        assert frame.detection_ratio == pytest.approx(0.0)

    def test_empty_lidar(self):
        radar = _xyz([(1, 0, 0)])
        frame = fuse_pair(radar, None, np.zeros((0, 3), np.float32))
        assert frame.n_lidar == 0
        assert math.isnan(frame.detection_ratio)

    def test_velocity_mean(self):
        pts = _xyz([(1, 0, 0), (2, 0, 0)])
        vel = np.array([3.0, 5.0], dtype=np.float32)
        frame = fuse_pair(pts, vel, pts)
        assert frame.mean_radar_velocity == pytest.approx(4.0)

    def test_velocity_absolute_value(self):
        """Negative velocities (approaching) should be counted by magnitude."""
        pts = _xyz([(1, 0, 0)])
        vel = np.array([-6.0], dtype=np.float32)
        frame = fuse_pair(pts, vel, pts)
        assert frame.mean_radar_velocity == pytest.approx(6.0)

    def test_velocity_none_gives_nan(self):
        pts = _xyz([(1, 0, 0)])
        frame = fuse_pair(pts, None, pts)
        assert math.isnan(frame.mean_radar_velocity)

    def test_counts(self):
        radar = _xyz([(1, 0, 0)] * 7)
        lidar = _xyz([(2, 0, 0)] * 3)
        frame = fuse_pair(radar, None, lidar)
        assert frame.n_radar == 7
        assert frame.n_lidar == 3

    def test_coverage_threshold_sensitivity(self):
        """A point 1 m away should be covered with threshold=2 but not threshold=0.5."""
        radar = _xyz([(1, 0, 0)])
        lidar = _xyz([(2, 0, 0)])   # 1 m apart in X
        frame_loose = fuse_pair(radar, None, lidar, coverage_threshold=2.0)
        frame_tight = fuse_pair(radar, None, lidar, coverage_threshold=0.5)
        assert frame_loose.coverage_pct_covered == pytest.approx(1.0)
        assert frame_tight.coverage_pct_covered == pytest.approx(0.0)


class TestSaveMetricsCsv:
    def _make_frames(self, n=3):
        return [
            FusionFrame(
                timestamp_ns=i * 100_000_000,
                n_radar=10 + i,
                n_lidar=50,
                detection_ratio=(10 + i) / 50,
                coverage_complement_m=0.3 + i * 0.1,
                coverage_pct_covered=0.8,
                mean_radar_velocity=1.5 + i * 0.2,
            )
            for i in range(n)
        ]

    def test_creates_csv(self):
        frames = self._make_frames()
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, "out.csv")
            save_metrics_csv(frames, path)
            assert os.path.isfile(path)

    def test_row_count(self):
        frames = self._make_frames(5)
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, "out.csv")
            save_metrics_csv(frames, path)
            with open(path) as f:
                rows = list(csv.DictReader(f))
            assert len(rows) == 5

    def test_header_fields(self):
        frames = self._make_frames(1)
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, "out.csv")
            save_metrics_csv(frames, path)
            with open(path) as f:
                reader = csv.DictReader(f)
                assert "detection_ratio" in reader.fieldnames
                assert "coverage_complement_m" in reader.fieldnames
                assert "coverage_pct_covered" in reader.fieldnames
                assert "mean_radar_velocity" in reader.fieldnames

    def test_nan_frames_written_as_nan(self):
        frame = FusionFrame(
            timestamp_ns=0, n_radar=0, n_lidar=5,
            detection_ratio=float("nan"),
            coverage_complement_m=float("nan"),
            coverage_pct_covered=float("nan"),
            mean_radar_velocity=float("nan"),
        )
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, "out.csv")
            save_metrics_csv([frame], path)
            with open(path) as f:
                row = list(csv.DictReader(f))[0]
            assert row["detection_ratio"] == "nan"


class TestPlotFusionSummary:
    def _make_frames(self):
        return [
            FusionFrame(
                timestamp_ns=i * 100_000_000,
                n_radar=10, n_lidar=50,
                detection_ratio=0.2,
                coverage_complement_m=0.4,
                coverage_pct_covered=0.75,
                mean_radar_velocity=2.0,
            )
            for i in range(10)
        ]

    def test_saves_png(self):
        frames = self._make_frames()
        with tempfile.TemporaryDirectory() as td:
            path = plot_fusion_summary(frames, td)
            assert os.path.isfile(path)
            assert path.endswith(".png")

    def test_empty_frames_does_not_crash(self):
        with tempfile.TemporaryDirectory() as td:
            plot_fusion_summary([], td)   # should not raise
