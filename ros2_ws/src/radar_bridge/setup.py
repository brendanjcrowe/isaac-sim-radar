from setuptools import find_packages, setup
import os
from glob import glob

package_name = "radar_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Brendan",
    maintainer_email="brendan@todo.com",
    description="Isaac Sim RTX Radar UDP to ROS2 PointCloud2 bridge",
    license="MIT",
    entry_points={
        "console_scripts": [
            "radar_to_ros2 = radar_bridge.radar_to_ros2:main",
        ],
    },
)
