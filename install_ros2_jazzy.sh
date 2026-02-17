#!/usr/bin/env bash
# install_ros2_jazzy.sh — Install ROS2 Jazzy on Ubuntu 24.04 and build the workspace
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="jazzy"

info()  { echo -e "\033[1;34m[INFO]\033[0m  $*"; }
error() { echo -e "\033[1;31m[ERROR]\033[0m $*"; exit 1; }

# ---------- 1. Check OS ----------
if [[ "$(lsb_release -cs 2>/dev/null)" != "noble" ]]; then
    error "This script targets Ubuntu 24.04 (noble). Detected: $(lsb_release -cs)"
fi

# ---------- 2. Install ROS2 Jazzy ----------
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    info "ROS2 ${ROS_DISTRO} already installed — skipping."
else
    info "Installing ROS2 ${ROS_DISTRO}..."

    sudo apt-get update
    sudo apt-get install -y software-properties-common curl

    # Add ROS2 GPG key
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    # Add repository
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu noble main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt-get update
    sudo apt-get install -y \
        ros-${ROS_DISTRO}-desktop \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-sensor-msgs \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-slam-toolbox \
        python3-colcon-common-extensions \
        python3-rosdep

    info "ROS2 ${ROS_DISTRO} installed."
fi

# ---------- 3. Initialize rosdep ----------
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    info "Initializing rosdep..."
    sudo rosdep init || true
fi
rosdep update || true

# ---------- 4. Build ROS2 workspace ----------
info "Building ROS2 workspace..."

# ROS2 setup scripts use unbound variables — temporarily relax strict mode
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

cd "${SCRIPT_DIR}/ros2_ws"
colcon build --symlink-install

set +u
source install/setup.bash
set -u

# ---------- 5. Verify ----------
info "Verifying installation..."
echo "  ROS_DISTRO=${ROS_DISTRO}"
echo "  ROS2 packages:"
ros2 pkg list 2>/dev/null | grep -E "radar_bridge|sensor_msgs|tf2_ros" | sed 's/^/    /'

info "Done! To use in a new terminal, run:"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  source ${SCRIPT_DIR}/ros2_ws/install/setup.bash"
