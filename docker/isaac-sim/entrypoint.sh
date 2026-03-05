#!/bin/bash
set -e

# Isaac Sim 5.1 bundles its own ROS2 Jazzy libs inside the isaacsim.ros2.bridge extension.
# Without this, the ROS2 bridge extension fails to start with "No such file or directory".
JAZZY_LIB=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib
if [ -d "$JAZZY_LIB" ]; then
    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$JAZZY_LIB"
    export ROS_DISTRO=jazzy
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
fi

if [ $# -gt 0 ]; then
    exec "$@"
fi

exec /isaac-sim/python.sh /app/isaac_sim_scripts/run_headless.py
