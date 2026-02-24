#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=== Isaac Sim Radar - Docker Start ==="

# Check NVIDIA runtime
if ! docker info 2>/dev/null | grep -q "nvidia"; then
    echo "WARNING: NVIDIA Docker runtime not detected."
    echo "Install nvidia-container-toolkit: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html"
fi

# Check GPU availability
if ! nvidia-smi &>/dev/null; then
    echo "ERROR: nvidia-smi not found. NVIDIA GPU driver required."
    exit 1
fi

# Check NGC authentication for Isaac Sim image
if [ -z "$NGC_API_KEY" ] && ! grep -q "nvcr.io" ~/.docker/config.json 2>/dev/null; then
    echo "WARNING: NGC authentication may be required to pull Isaac Sim image."
    echo "Run: docker login nvcr.io --username '\$oauthtoken' --password <NGC_API_KEY>"
fi

# Allow X11 forwarding for RViz
if command -v xhost &>/dev/null; then
    xhost +local:docker 2>/dev/null || true
fi

# Create output directories
mkdir -p ../bags ../analysis_output

echo "Starting containers..."
docker compose up --build -d

echo ""
echo "=== Containers started ==="
echo ""
echo "Useful commands:"
echo "  docker compose -f docker/docker-compose.yaml logs -f        # Follow logs"
echo "  docker exec isaac-sim nvidia-smi                            # Check GPU"
echo "  docker exec ros2-bridge ros2 topic list                     # List ROS2 topics"
echo "  docker exec ros2-bridge ros2 run radar_bridge radar_to_ros2 # Start radar bridge"
echo "  docker exec ros2-bridge pytest /app/tests/ -v               # Run tests"
echo "  ./docker/stop.sh                                            # Stop all"
