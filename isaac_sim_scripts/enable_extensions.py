"""Enable required Isaac Sim extensions programmatically.

Run this inside Isaac Sim's Script Editor or via the --exec flag:
    ./isaac-sim.sh --exec enable_extensions.py
"""

import omni.kit.app

REQUIRED_EXTENSIONS = [
    "omni.sensors.nv.radar",
    "omni.sensors.nv.common",
    "omni.isaac.ros2_bridge",
    "omni.isaac.sensor",
    "omni.sensors.nv.lidar",
]


def enable_extensions():
    """Enable all required extensions for the radar project."""
    manager = omni.kit.app.get_app().get_extension_manager()

    for ext_id in REQUIRED_EXTENSIONS:
        if not manager.is_extension_enabled(ext_id):
            result = manager.set_extension_enabled_immediate(ext_id, True)
            if result:
                print(f"[OK] Enabled: {ext_id}")
            else:
                print(f"[FAIL] Could not enable: {ext_id}")
        else:
            print(f"[OK] Already enabled: {ext_id}")


if __name__ == "__main__":
    enable_extensions()
