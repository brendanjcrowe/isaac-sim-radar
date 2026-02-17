#!/usr/bin/env bash
# setup_env.sh — Bootstrap the Isaac Sim Radar project environment
# Usage: bash setup_env.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="jazzy"

# ---------- helpers ----------
info()  { echo -e "\033[1;34m[INFO]\033[0m  $*"; }
warn()  { echo -e "\033[1;33m[WARN]\033[0m  $*"; }
error() { echo -e "\033[1;31m[ERROR]\033[0m $*"; exit 1; }

confirm() {
    read -rp "$1 [Y/n] " ans
    case "${ans,,}" in
        ""|y|yes) return 0 ;;
        *) return 1 ;;
    esac
}

# ---------- 1. ROS2 Jazzy ----------
install_ros2() {
    if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
        info "ROS2 ${ROS_DISTRO} already installed."
        return 0
    fi

    info "Installing ROS2 ${ROS_DISTRO}..."
    confirm "This will run sudo commands to install ROS2. Continue?" || return 0

    sudo apt-get update && sudo apt-get install -y software-properties-common curl

    # Add ROS2 GPG key
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    # Add repository
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
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

    # Initialize rosdep if needed
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init || true
    fi
    rosdep update || true

    info "ROS2 ${ROS_DISTRO} installed successfully."
}

# ---------- 2. Python dependencies ----------
install_python_deps() {
    info "Installing Python dependencies..."
    pip install --user -r "${SCRIPT_DIR}/requirements.txt"
    info "Python dependencies installed."
}

# ---------- 3. Build ROS2 workspace ----------
build_ros2_workspace() {
    info "Building ROS2 workspace..."

    if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
        warn "ROS2 ${ROS_DISTRO} not found — skipping workspace build."
        return 0
    fi

    # shellcheck disable=SC1091
    source "/opt/ros/${ROS_DISTRO}/setup.bash"

    cd "${SCRIPT_DIR}/ros2_ws"
    colcon build --symlink-install
    info "ROS2 workspace built successfully."
    cd "${SCRIPT_DIR}"
}

# ---------- 4. Isaac Sim instructions ----------
print_isaac_sim_instructions() {
    echo ""
    info "=========================================="
    info " Manual Step: Install Isaac Sim 4.5"
    info "=========================================="
    echo ""
    echo "  1. Download Omniverse Launcher from:"
    echo "     https://www.nvidia.com/en-us/omniverse/download/"
    echo ""
    echo "  2. Make it executable and run:"
    echo "     chmod +x omniverse-launcher-linux*.AppImage"
    echo "     ./omniverse-launcher-linux*.AppImage"
    echo ""
    echo "  3. In the Launcher, go to Exchange → Isaac Sim → Install (version 4.5.0)"
    echo ""
    echo "  4. Verify it launches and detects your GPU."
    echo ""
    echo "  5. Enable these extensions in Window → Extensions:"
    echo "     - omni.sensors.nv.radar"
    echo "     - omni.sensors.nv.common"
    echo "     - omni.isaac.ros2_bridge"
    echo "     - omni.isaac.sensor"
    echo ""
}

# ---------- main ----------
main() {
    info "Isaac Sim Radar — Environment Setup"
    echo ""

    install_ros2
    install_python_deps
    build_ros2_workspace
    print_isaac_sim_instructions

    echo ""
    info "Setup complete! Source your environment with:"
    echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
    echo "  source ${SCRIPT_DIR}/ros2_ws/install/setup.bash"
}

main "$@"
