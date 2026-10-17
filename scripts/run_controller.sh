#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=42 UAV_SIMULATION_ONLY=1
source ros2_ws/install/setup.bash
exec ros2 run uav_controller sitl_mission --ros-args --params-file config/controller.yaml
