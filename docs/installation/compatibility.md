# Compatibility decision — reviewed 2026-10-15

| Dependency | Selected target | Pin / evidence |
|---|---|---|
| OS | Ubuntu 22.04 LTS | PX4 v1.16 ROS guide recommends Humble on Jammy |
| ROS 2 | Humble | do not install Humble binaries on initial Noble host |
| PX4 | v1.16.0 | exact git tag checked by launcher |
| Gazebo | Harmonic / gz-sim8 | PX4 v1.16 Ubuntu toolchain; not Humble default Fortress |
| ROS-Gazebo bridge | ros-humble-ros-gzharmonic | camera/clock integration deferred |
| Agent | Micro-XRCE-DDS-Agent v2.4.3 | version in official PX4 v1.16 ROS guide |
| px4_msgs | release/1.16 scaffold + firmware-exact messages | sync script records commit SHAs |
| C++ | C++20, GCC 11+ | standalone host GCC recorded in test report |
| Node | 22.14.0 target | dashboard dependency lock committed |
| Python | 3.10 target; 3.12 initial experiments | target-host versions must be independently tested |

This is a supported-family decision, not a tested installation lock. OS/ROS/Gazebo package patch versions must be
recorded with `dpkg-query -W` on the actual simulation host. Dependencies pulled by the PX4 installer may change;
archive the resolved package list and source SHAs with each regression run.
The optional PX4 ROS 2 Interface Library is deferred; direct px4_msgs avoids adding another compatibility dependency.
Eigen, GoogleTest, ONNX Runtime, OpenCV and PyTorch are not integrated in 0.1; see roadmap.

Official sources:
- https://docs.px4.io/v1.16/en/dev_setup/dev_env_linux_ubuntu
- https://docs.px4.io/v1.16/en/ros2/user_guide
- https://docs.px4.io/v1.16/en/ros2/offboard_control
- https://docs.px4.io/v1.16/en/flight_modes/offboard
- https://docs.px4.io/v1.16/en/advanced_config/parameter_reference
- https://gazebosim.org/docs/harmonic/install_ubuntu/
- https://github.com/PX4/px4_msgs
