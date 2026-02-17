"""Tests for config file loading and validity."""

import os
import unittest
import yaml

CONFIG_DIR = os.path.join(os.path.dirname(__file__), "..", "config")


class TestConfigFiles(unittest.TestCase):

    def _load(self, name):
        path = os.path.join(CONFIG_DIR, name)
        self.assertTrue(os.path.exists(path), f"{name} not found")
        with open(path) as f:
            return yaml.safe_load(f)

    def test_radar_params(self):
        cfg = self._load("radar_params.yaml")
        self.assertEqual(cfg["model"], "WpmDmatApproxRadar")
        self.assertIn("range", cfg)
        self.assertIn("velocity", cfg)
        self.assertIn("fov", cfg)
        self.assertIn("udp", cfg)
        self.assertEqual(cfg["udp"]["remote_port"], 10001)
        self.assertGreater(cfg["range"]["max_range_m"], 0)

    def test_lidar_params(self):
        cfg = self._load("lidar_params.yaml")
        self.assertIn("config_name", cfg)
        self.assertIn("mount_offset", cfg)
        self.assertIn("ros2_topic", cfg)

    def test_robot_sensors(self):
        cfg = self._load("robot_sensors.yaml")
        self.assertIn("frames", cfg)
        frames = cfg["frames"]
        self.assertIn("radar_link", frames)
        self.assertIn("lidar_link", frames)
        self.assertIn("base_link", frames)
        # Check radar mount has valid translation
        radar = frames["radar_link"]
        self.assertEqual(len(radar["translation"]), 3)

    def test_radar_bridge_config(self):
        path = os.path.join(os.path.dirname(__file__), "..",
                            "ros2_ws", "src", "radar_bridge", "config",
                            "radar_bridge.yaml")
        self.assertTrue(os.path.exists(path))
        with open(path) as f:
            cfg = yaml.safe_load(f)
        params = cfg["radar_to_ros2"]["ros__parameters"]
        self.assertEqual(params["multicast_group"], "239.0.0.1")
        self.assertEqual(params["port"], 10001)
        self.assertEqual(params["frame_id"], "radar_link")


if __name__ == "__main__":
    unittest.main()
