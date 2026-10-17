# navigation_controller
Implemented as a header component at `core/include/uav/navigation_controller.hpp` and exported by the `uav_core` ament package.
`MissionController` composes this module for both the standalone harness and the `uav_controller` ROS adapter.
This directory documents the architecture mapping; it is not a separate ROS node/package.
Build from the repository root with CMake, or use `colcon build --packages-up-to uav_controller` in ros2_ws.
