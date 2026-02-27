"""Tests for radar_analysis/radar_map.py."""

import math
import os
import sys
import tempfile

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from radar_analysis.radar_map import RadarOccupancyGrid


class TestRadarOccupancyGridInit:
    def test_grid_shape(self):
        g = RadarOccupancyGrid(resolution=1.0, x_min=0, x_max=10, y_min=0, y_max=5)
        assert g.grid.shape == (5, 10)

    def test_grid_starts_at_zero(self):
        g = RadarOccupancyGrid()
        assert g.grid.sum() == 0


class TestAccumulate:
    def test_identity_transform_places_correctly(self):
        """Detection at (2, 1) with robot at origin should land in that cell."""
        g = RadarOccupancyGrid(resolution=1.0, x_min=0, x_max=10, y_min=0, y_max=5)
        g.accumulate(np.array([2.0]), np.array([1.0]))
        # cell (row=1, col=2)
        assert g.grid[1, 2] == 1

    def test_translation(self):
        """Robot translated to (5, 2): detection at sensor (0,0) lands at map (5,2)."""
        g = RadarOccupancyGrid(resolution=1.0, x_min=0, x_max=10, y_min=0, y_max=5)
        g.accumulate(np.array([0.0]), np.array([0.0]), robot_x=5.0, robot_y=2.0)
        assert g.grid[2, 5] == 1

    def test_rotation_90_degrees(self):
        """Robot yawed 90°: sensor +X maps to odom +Y."""
        g = RadarOccupancyGrid(resolution=1.0, x_min=-5, x_max=5, y_min=-5, y_max=5)
        g.accumulate(
            np.array([3.0]), np.array([0.0]),
            robot_yaw=math.pi / 2,
        )
        # After 90° rotation: x_map ≈ 0, y_map ≈ 3
        col = int((0.0 - (-5)) / 1.0)
        row = int((3.0 - (-5)) / 1.0)
        assert g.grid[row, col] == 1

    def test_rcs_threshold_filters_weak_returns(self):
        """Detections below rcs_threshold should be ignored."""
        g = RadarOccupancyGrid(
            resolution=1.0, x_min=0, x_max=10, y_min=0, y_max=5,
            rcs_threshold=-10.0,
        )
        xs  = np.array([1.0, 2.0])
        ys  = np.array([0.0, 0.0])
        rcs = np.array([-20.0, -5.0])   # first below threshold, second above
        count = g.accumulate(xs, ys, rcs=rcs)
        assert count == 1
        assert g.grid.sum() == 1

    def test_out_of_bounds_ignored(self):
        g = RadarOccupancyGrid(resolution=1.0, x_min=0, x_max=5, y_min=0, y_max=5)
        g.accumulate(np.array([100.0]), np.array([100.0]))
        assert g.grid.sum() == 0

    def test_multiple_frames_accumulate(self):
        g = RadarOccupancyGrid(resolution=1.0, x_min=0, x_max=10, y_min=0, y_max=5)
        for _ in range(5):
            g.accumulate(np.array([1.0]), np.array([1.0]))
        assert g.grid[1, 1] == 5

    def test_returns_count(self):
        g = RadarOccupancyGrid(resolution=1.0, x_min=0, x_max=10, y_min=0, y_max=5)
        count = g.accumulate(np.array([1.0, 2.0, 3.0]), np.array([0.0, 0.0, 0.0]))
        assert count == 3

    def test_empty_arrays(self):
        g = RadarOccupancyGrid()
        count = g.accumulate(np.array([]), np.array([]))
        assert count == 0
        assert g.grid.sum() == 0


class TestSaveMap:
    def test_save_creates_three_files(self):
        g = RadarOccupancyGrid(resolution=1.0, x_min=0, x_max=5, y_min=0, y_max=5)
        g.accumulate(np.array([2.0]), np.array([2.0]))
        with tempfile.TemporaryDirectory() as td:
            png, pgm, yml = g.save_map(td)
            assert os.path.isfile(png)
            assert os.path.isfile(pgm)
            assert os.path.isfile(yml)

    def test_yaml_contains_resolution(self):
        g = RadarOccupancyGrid(resolution=0.5, x_min=0, x_max=5, y_min=0, y_max=5)
        with tempfile.TemporaryDirectory() as td:
            _, _, yml = g.save_map(td)
            content = open(yml).read()
            assert "resolution: 0.5" in content

    def test_yaml_contains_origin(self):
        g = RadarOccupancyGrid(resolution=1.0, x_min=-3.0, x_max=5.0, y_min=-2.0, y_max=2.0)
        with tempfile.TemporaryDirectory() as td:
            _, _, yml = g.save_map(td)
            content = open(yml).read()
            assert "-3.000" in content
            assert "-2.000" in content

    def test_pgm_header(self):
        g = RadarOccupancyGrid(resolution=1.0, x_min=0, x_max=4, y_min=0, y_max=3)
        with tempfile.TemporaryDirectory() as td:
            _, pgm, _ = g.save_map(td)
            with open(pgm, "rb") as f:
                header = f.read(20)
            assert header.startswith(b"P5\n")

    def test_empty_grid_save_succeeds(self):
        g = RadarOccupancyGrid(resolution=1.0, x_min=0, x_max=5, y_min=0, y_max=5)
        with tempfile.TemporaryDirectory() as td:
            png, pgm, yml = g.save_map(td)
            assert os.path.isfile(png)
