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

# Start a virtual X11 display if none is available. This changes the carb plugin
# initialization order so that IMaterialReaderFactory / IProfileReaderFactory are
# registered before the RTX radar model plugins (wpm_dmatapprox, dmatapprox) start.
# xvfb-run relies on SIGUSR1 from Xvfb which does not work in all container
# environments, so we start Xvfb manually and poll xdpyinfo instead.
if [ -z "${DISPLAY}" ] && command -v Xvfb >/dev/null 2>&1; then
    DISPLAY=:99
    export DISPLAY
    # Remove stale lock file left by a previous container run (docker restart
    # preserves /tmp in the container's writable layer, so the old lock survives).
    rm -f /tmp/.X99-lock
    Xvfb :99 -screen 0 1280x1024x24 -nolisten tcp &
    XVFB_PID=$!
    # Wait up to 10s for the display to become ready
    for _i in $(seq 1 20); do
        xdpyinfo -display :99 >/dev/null 2>&1 && break
        sleep 0.5
    done
    echo "[entrypoint] Xvfb started on :99 (PID $XVFB_PID)"
fi

if [ $# -gt 0 ]; then
    exec "$@"
fi

exec /isaac-sim/python.sh /app/isaac_sim_scripts/run_headless.py
