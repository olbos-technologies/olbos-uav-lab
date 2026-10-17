# Runbook

## Standalone development (works without ROS)
From the extracted repository root, with g++ and Python 3 available:
```bash
bash scripts/test_core.sh
bash scripts/run_demo.sh
python3 -m unittest discover -s tests/integration -v
```
Expected: 25 checks pass; seven Python integration tests pass (six harness scenarios plus replay). `artifacts/flight.jsonl` contains kinematic data, not PX4 telemetry.
For the GoogleTest mission suite (install libgtest-dev on Ubuntu):
```bash
cmake -S . -B build -DUAV_BUILD_GTEST=ON
cmake --build build
ctest --test-dir build --output-on-failure --output-junit ctest-results.xml
```
Expected: 17 CTest cases pass (16 GoogleTests and the 25-check core contract executable).
The optional GoogleTest flag is off by default for a minimal dependency-free core build; CI enables it.

Use a Python virtual environment:
```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r machine_learning/requirements.txt
python3 machine_learning/training/train.py
python3 scripts/telemetry_ws.py artifacts/flight.jsonl --speed 1
```
Expected: synthetic evaluation JSON and locally generated model files under artifacts; replay server on loopback port 8765.
In another terminal:
```bash
cd dashboard
npm ci
npm run dev
```
Open http://127.0.0.1:5173. Start replay after opening the dashboard, or refresh while replay runs (each connection replays independently).
The browser must run on the same host or use an explicitly configured SSH loopback port forward.
The UI starts with no data; no mock values masquerade as live telemetry. Export contains only its latest 600 samples.

## PX4 host setup (not executed in initial environment)
Use a dedicated Ubuntu 22.04 VM/native installation with GUI support. WSL2 requires working WSLg/OpenGL and additional Gazebo validation.
Install ROS Humble using the official ROS instructions linked in compatibility.md. Do not substitute unrelated ROS distributions.
Install `git`, `cmake`, `build-essential`, `python3-colcon-common-extensions` and ROS development tools.

```bash
mkdir -p .external
git clone --recursive --branch v1.16.0 --depth 1 https://github.com/PX4/PX4-Autopilot.git .external/PX4-Autopilot
bash .external/PX4-Autopilot/Tools/setup/ubuntu.sh
```
The official installer uses sudo and installs system packages; inspect it before executing. Reboot if it asks.

```bash
git clone --branch v2.4.3 --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git .external/Micro-XRCE-DDS-Agent
cmake -S .external/Micro-XRCE-DDS-Agent -B .external/agent-build
cmake --build .external/agent-build -j2
sudo cmake --install .external/agent-build
sudo ldconfig
source /opt/ros/humble/setup.bash
git clone --branch release/1.16 --depth 1 https://github.com/PX4/px4_msgs.git ros2_ws/src/px4_msgs
python3 scripts/sync_px4_messages.py "$PWD/.external/PX4-Autopilot"
cd ros2_ws
colcon build --packages-up-to uav_controller
cd ..
```
Expected: px4_msgs, uav_core and uav_controller build. Message generation/schema compatibility remains unverified here.

Run terminals on an isolated VM/network with no physical autopilot devices, and set `ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1` in every ROS terminal.
Start agent: `MicroXRCEAgent udp4 -p 8888`. Agent UDP may listen on all interfaces; VM/firewall isolation is required.
Start simulator: `PX4_DIR="$PWD/.external/PX4-Autopilot" bash scripts/launch_sitl.sh`.
Use the PX4 shell checklist in `px4/configuration/sitl-checklist.md`, including DDS domain and Land-on-loss settings.
Keep QGroundControl connected. Inspect `ros2 topic list` and `ros2 topic hz` for position/status/battery/land topics before commanding.
Then `bash scripts/run_controller.sh`.
Expected (not observed here): REQUEST/ACK logs, climb to 5 m, Land request, then `STATE COMPLETE outcome=1` only after fresh landed/disarmed observations.
If telemetry is unavailable, no arm request should be sent and the node faults at its 90 second deadline.
The process remains running after Complete/Fault for inspection; Ctrl+C exits. Emergency stops rely on configured PX4 failsafes.
Capture PX4 ULog and controller stdout as acceptance evidence. Never report COMPLETE from ACK alone.

Live read-only dashboard: source `/opt/ros/humble/setup.bash` and `ros2_ws/install/setup.bash`, set domain/local-only variables,
and run `python3 scripts/live_telemetry_ws.py`. Use the system Python/venv with access to rclpy and installed websockets.
Do not run live and replay servers simultaneously; both use port 8765.

## Optional custom world
`simulation/worlds/basic.sdf` can be checked with `gz sdf -k simulation/worlds/basic.sdf` on the target host.
It is not the default PX4 world and has not been Gazebo-validated here. First establish flight in the shipped PX4 default world;
then add the custom world through PX4's documented GZ_SIM_RESOURCE_PATH/PX4_GZ_WORLD mechanism.

## Shared controller waypoint and abort commands
After the smoke mission passes on the isolated SITL target, run the same engine with the square route:
```bash
bash scripts/run_controller.sh config/waypoint-mission.yaml
```
From a ROS terminal with the same domain, abort an active mission:
```bash
ros2 service call /sitl_mission/abort std_srvs/srv/Trigger '{}'
```
Expected service response: abort queued, PX4 landing unconfirmed. Observe subsequent controller state and PX4 ULog.
These ROS commands are source/runbook deliverables; they have not been executed in the current environment.
