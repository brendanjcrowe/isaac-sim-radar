#!/bin/bash
# Post-start smoke test for the Isaac Sim Radar stack.
# Run this after docker/start.sh to verify each layer is healthy.
# Exits non-zero on the first failed check.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PASS=0
FAIL=0

ok()   { echo "  [PASS] $1"; ((PASS++)); }
fail() { echo "  [FAIL] $1"; ((FAIL++)); }

header() { echo ""; echo "=== $1 ==="; }

# ── Check 1: GPU visible in isaac-sim ────────────────────────────────────────
header "1/5  GPU in isaac-sim container"
if docker exec isaac-sim nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | grep -q .; then
    ok "nvidia-smi reports GPU: $(docker exec isaac-sim nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)"
else
    fail "nvidia-smi failed or found no GPU in isaac-sim container"
    exit 1
fi

# ── Check 2: radar_bridge package installed ──────────────────────────────────
header "2/5  radar_bridge package in ros2-bridge"
if docker exec ros2-bridge bash -c \
    "source /opt/ros/humble/setup.bash && \
     source /app/ros2_ws/install/setup.bash 2>/dev/null && \
     ros2 pkg list | grep -q radar_bridge"; then
    ok "radar_bridge found in ros2 pkg list"
else
    fail "radar_bridge not found — workspace may not be built"
    exit 1
fi

# ── Check 3: unit + integration tests pass ───────────────────────────────────
header "3/5  pytest inside ros2-bridge"
if docker exec ros2-bridge bash -c \
    "cd /app && python3 -m pytest tests/ -q --tb=short 2>&1"; then
    ok "All tests passed inside container"
else
    fail "Tests failed inside container"
    exit 1
fi

# ── Check 4: analysis modules importable ─────────────────────────────────────
header "4/5  radar_analysis imports"
if docker exec ros2-bridge python3 -c \
    "import radar_analysis.compare_clouds; \
     import radar_analysis.radar_pointcloud; \
     import radar_analysis.parse_radar_udp; \
     print('imports OK')"; then
    ok "radar_analysis modules import cleanly"
else
    fail "radar_analysis import failed"
    exit 1
fi

# ── Check 5: node starts without immediate crash ─────────────────────────────
header "5/5  radar_to_ros2 node starts"
docker exec ros2-bridge bash -c "
    source /opt/ros/humble/setup.bash
    source /app/ros2_ws/install/setup.bash 2>/dev/null
    ros2 run radar_bridge radar_to_ros2 &
    NODE_PID=\$!
    sleep 3
    if kill -0 \$NODE_PID 2>/dev/null; then
        kill \$NODE_PID 2>/dev/null
        echo 'node_running'
    else
        echo 'node_crashed'
    fi
" | grep -q node_running && ok "radar_to_ros2 node ran for 3s without crashing" \
                          || { fail "radar_to_ros2 node crashed at startup"; exit 1; }

# ── Summary ──────────────────────────────────────────────────────────────────
echo ""
echo "======================================="
echo "  Results: ${PASS} passed, ${FAIL} failed"
echo "======================================="
echo ""
echo "Next steps (requires Isaac Sim running):"
echo "  1. Inside isaac-sim container, run the scene:"
echo "       docker exec isaac-sim /isaac-sim/python.sh /app/isaac_sim_scripts/launch_scene.py"
echo "  2. Press Play in Isaac Sim (or wait for headless auto-play)"
echo "  3. Check ROS2 topics appear:"
echo "       docker exec ros2-bridge bash -c 'source /opt/ros/humble/setup.bash && ros2 topic list'"
echo "  4. Record a bag:"
echo "       docker exec ros2-bridge python3 /app/radar_analysis/record_bags.py --duration 30"
echo ""
