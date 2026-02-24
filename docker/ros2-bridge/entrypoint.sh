#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

# Rebuild workspace if bind-mounted source is present (fast for Python-only)
if [ -d /app/ros2_ws/src ]; then
    cd /app/ros2_ws
    colcon build --symlink-install 2>/dev/null || true
    cd /app
fi

# Source workspace overlay
if [ -f /app/ros2_ws/install/setup.bash ]; then
    source /app/ros2_ws/install/setup.bash
fi

exec "$@"
