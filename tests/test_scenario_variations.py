"""Tests for scenario variation logic in isaac_sim_scripts/launch_scene.py.

The Isaac Sim API (omni.*, carb.*) is not available in this environment.
These tests cover the pure-Python logic that can be validated offline:
  - Triangle-wave interpolation used by add_dynamic_objects
  - Fog preset parameter table
  - Pedestrian/vehicle path definitions (geometry sanity checks)
  - run_headless.py CLI argument parsing (no SimulationApp involved)
"""

import math
import os
import sys

import pytest

# ── Helpers that mirror the math in launch_scene.py ─────────────────────────

def _triangle_wave(frame: int, num_frames: int) -> float:
    """Triangle-wave alpha used for pedestrian oscillation.

    Returns 0.0 at frame 0, 1.0 at frame num_frames//2, 0.0 at frame num_frames.
    Mirrors the logic in add_dynamic_objects().
    """
    phase = (frame / num_frames) * 2.0
    return phase if phase <= 1.0 else 2.0 - phase


def _interp(start, end, alpha):
    return start + alpha * (end - start)


# ── Pedestrian animation math ────────────────────────────────────────────────

class TestTriangleWave:
    def test_starts_at_zero(self):
        assert _triangle_wave(0, 600) == pytest.approx(0.0)

    def test_midpoint_at_one(self):
        assert _triangle_wave(300, 600) == pytest.approx(1.0)

    def test_ends_at_zero(self):
        assert _triangle_wave(600, 600) == pytest.approx(0.0)

    def test_first_quarter(self):
        alpha = _triangle_wave(150, 600)
        assert 0.0 < alpha < 1.0

    def test_monotone_ascending_first_half(self):
        alphas = [_triangle_wave(f, 600) for f in range(0, 301, 30)]
        for a, b in zip(alphas, alphas[1:]):
            assert b >= a

    def test_monotone_descending_second_half(self):
        alphas = [_triangle_wave(f, 600) for f in range(300, 601, 30)]
        for a, b in zip(alphas, alphas[1:]):
            assert b <= a

    def test_symmetric_around_midpoint(self):
        for f in range(0, 301, 50):
            assert _triangle_wave(f, 600) == pytest.approx(_triangle_wave(600 - f, 600))


class TestPedestrianInterpolation:
    """Verify that pedestrian positions lie on the expected linear segment."""

    start = (15.0, 4.0, 0.9)
    end   = (15.0, -4.0, 0.9)   # pure Y motion

    def _pos(self, frame, num_frames=600):
        alpha = _triangle_wave(frame, num_frames)
        sx, sy, sz = self.start
        ex, ey, ez = self.end
        return (
            _interp(sx, ex, alpha),
            _interp(sy, ey, alpha),
            sz,
        )

    def test_initial_position_is_start(self):
        x, y, z = self._pos(0)
        assert x == pytest.approx(self.start[0])
        assert y == pytest.approx(self.start[1])

    def test_midpoint_position_is_end(self):
        x, y, z = self._pos(300)
        assert x == pytest.approx(self.end[0])
        assert y == pytest.approx(self.end[1])

    def test_final_position_is_start(self):
        x, y, z = self._pos(600)
        assert x == pytest.approx(self.start[0])
        assert y == pytest.approx(self.start[1])

    def test_x_stays_constant_pure_y_path(self):
        """This pedestrian only moves in Y — X should be constant."""
        xs = [self._pos(f)[0] for f in range(0, 601, 60)]
        assert all(x == pytest.approx(15.0) for x in xs)

    def test_position_within_bounds(self):
        y_min = min(self.start[1], self.end[1])
        y_max = max(self.start[1], self.end[1])
        for f in range(0, 601, 10):
            _, y, _ = self._pos(f)
            assert y_min - 1e-9 <= y <= y_max + 1e-9


class TestVehicleInterpolation:
    """Vehicle uses a simple linear (one-way) path."""

    start = (-5.0, -5.0, 0.75)
    end   = (60.0, -5.0, 0.75)

    def _pos(self, frame, num_frames=600):
        alpha = frame / num_frames
        sx, sy, sz = self.start
        ex, ey, ez = self.end
        return (_interp(sx, ex, alpha), sy, sz)

    def test_starts_at_start(self):
        x, y, z = self._pos(0)
        assert x == pytest.approx(self.start[0])

    def test_ends_at_end(self):
        x, y, z = self._pos(600)
        assert x == pytest.approx(self.end[0])

    def test_monotone_x(self):
        xs = [self._pos(f)[0] for f in range(0, 601, 60)]
        for a, b in zip(xs, xs[1:]):
            assert b >= a - 1e-9

    def test_y_constant(self):
        for f in range(0, 601, 60):
            _, y, _ = self._pos(f)
            assert y == pytest.approx(self.start[1])


# ── Fog preset table ──────────────────────────────────────────────────────────

class TestFogPresets:
    """Verify the fog preset parameter table has expected structure."""

    _PRESETS = {
        "light": {"start": 20.0, "end": 80.0},
        "dense": {"start":  5.0, "end": 30.0},
    }

    def test_both_presets_present(self):
        assert "light" in self._PRESETS
        assert "dense" in self._PRESETS

    def test_dense_starts_closer_than_light(self):
        assert self._PRESETS["dense"]["start"] < self._PRESETS["light"]["start"]

    def test_dense_ends_closer_than_light(self):
        assert self._PRESETS["dense"]["end"] < self._PRESETS["light"]["end"]

    def test_start_less_than_end(self):
        for name, p in self._PRESETS.items():
            assert p["start"] < p["end"], f"Preset '{name}' has start >= end"


# ── run_headless.py argparse ─────────────────────────────────────────────────
# We import the arg definitions without triggering SimulationApp by monkey-
# patching sys.argv and reading the parser before it calls parse_args().
# This is done by importing argparse and reimplementing the same parser here,
# which mirrors the actual run_headless.py parser.

def _build_parser():
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--duration",        type=float, default=0.0)
    p.add_argument("--no-ros2",         action="store_true")
    p.add_argument("--weather",         choices=["clear", "fog", "rain"], default="clear")
    p.add_argument("--fog-preset",      choices=["light", "dense"],       default="light")
    p.add_argument("--dynamic-objects", action="store_true")
    return p


class TestRunHeadlessArgs:
    def test_defaults(self):
        args = _build_parser().parse_args([])
        assert args.duration == pytest.approx(0.0)
        assert args.no_ros2 is False
        assert args.weather == "clear"
        assert args.fog_preset == "light"
        assert args.dynamic_objects is False

    def test_duration_flag(self):
        args = _build_parser().parse_args(["--duration", "120"])
        assert args.duration == pytest.approx(120.0)

    def test_no_ros2_flag(self):
        args = _build_parser().parse_args(["--no-ros2"])
        assert args.no_ros2 is True

    def test_weather_fog(self):
        args = _build_parser().parse_args(["--weather", "fog"])
        assert args.weather == "fog"

    def test_weather_rain(self):
        args = _build_parser().parse_args(["--weather", "rain"])
        assert args.weather == "rain"

    def test_fog_preset_dense(self):
        args = _build_parser().parse_args(["--weather", "fog", "--fog-preset", "dense"])
        assert args.fog_preset == "dense"

    def test_dynamic_objects_flag(self):
        args = _build_parser().parse_args(["--dynamic-objects"])
        assert args.dynamic_objects is True

    def test_invalid_weather_rejected(self):
        with pytest.raises(SystemExit):
            _build_parser().parse_args(["--weather", "snow"])

    def test_full_scenario_flags(self):
        args = _build_parser().parse_args([
            "--duration", "60",
            "--weather", "rain",
            "--dynamic-objects",
            "--no-ros2",
        ])
        assert args.duration == pytest.approx(60.0)
        assert args.weather == "rain"
        assert args.dynamic_objects is True
        assert args.no_ros2 is True
