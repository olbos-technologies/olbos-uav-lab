#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
# Loopback-only by default; an isolated CI VM may pass ROS_LOCALHOST_ONLY=0 (see scripts/sitl/ci_acceptance.sh).
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1} ROS_DOMAIN_ID=42 UAV_SIMULATION_ONLY=1
source ros2_ws/install/setup.bash
exec ros2 run uav_controller sitl_mission --ros-args --params-file "${1:-config/controller.yaml}"
