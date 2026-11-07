# Gazebo x500 SITL acceptance

The `gazebo-x500-sitl` workflow runs on a GitHub-hosted `ubuntu-22.04` runner with PX4 `v1.16.0`, ROS 2 Humble, Gazebo Harmonic (`gz-sim8`), and Micro XRCE-DDS Agent `v2.4.3`. The workflow runs only software-in-the-loop, uses the `gz_x500` model, and does not configure serial devices or forward MAVLink to a physical vehicle. The source refs and resolved apt package versions are saved with each evidence artifact.

## Developer run

Use an isolated Ubuntu 22.04 VM with no flight-controller USB/serial devices and no MAVLink forwarding. Install ROS 2 Humble using the official instructions, then prepare the PX4 source checkout:

```bash
mkdir -p .external
git clone --recursive --shallow-submodules --depth 1 --branch v1.16.0 \
  https://github.com/PX4/PX4-Autopilot.git .external/PX4-Autopilot
```

For Gazebo Harmonic on Jammy, add the OSRF Gazebo apt source and install its Harmonic metapackage:

```bash
curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  | sudo gpg --dearmor -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt-get update
sudo apt-get install -y gz-harmonic
bash .external/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
```

Build Micro XRCE-DDS Agent v2.4.3 and the ROS workspace against the exact PX4 source checkout:

```bash
git clone --depth 1 --branch v2.4.3 \
  https://github.com/eProsima/Micro-XRCE-DDS-Agent.git .external/Micro-XRCE-DDS-Agent
cmake -S .external/Micro-XRCE-DDS-Agent -B .external/agent-build \
  -DCMAKE_INSTALL_PREFIX="$HOME/agent-install"
cmake --build .external/agent-build -j2
cmake --install .external/agent-build
export PATH="$HOME/agent-install/bin:$PATH"
export LD_LIBRARY_PATH="$HOME/agent-install/lib:${LD_LIBRARY_PATH:-}"

source /opt/ros/humble/setup.bash
git clone --depth 1 --branch release/1.16 \
  https://github.com/PX4/px4_msgs.git ros2_ws/src/px4_msgs
python3 scripts/sync_px4_messages.py "$PWD/.external/PX4-Autopilot"
cd ros2_ws
colcon build --packages-up-to uav_controller --cmake-args -DCMAKE_BUILD_TYPE=Release
cd ..
```

Run the complete x500 suite; the script starts and stops the agent, PX4/Gazebo and each mission process:

```bash
PX4_DIR="$PWD/.external/PX4-Autopilot" \
  bash scripts/sitl/gazebo_acceptance.sh artifacts/gazebo-sitl
```

The script applies and records `UXRCE_DDS_DOM_ID=42`, `COM_OF_LOSS_T=1`, `COM_OBL_RC_ACT=4`, and the CI-only no-RC setting `COM_RC_IN_MODE=4`. Do not disable arming checks. Verify parameter meanings against the checked-out PX4 release before adapting the run to another version.

## Required invariants and evidence

Smoke and waypoint missions and the operator abort are each repeated twice. Each run must have the expected controller outcome and reason, a recorded controller Land request, and a separate read of PX4 `VehicleStatus` and `VehicleLandDetected` reporting disarmed and landed. A command acknowledgement alone is never evidence of mission completion.

Fault cases run once each and must be detected by the controller, stop normal setpoint streaming through the shared safety controller, issue its Land fallback, and end with actual PX4 landed/disarmed observations:

| Case | Injection boundary | Expected controller outcome |
|---|---|---|
| GNSS-loss observation | ROS adapter marks controller-facing position invalid | `ABORTED_LANDED / INVALID_POSITION` |
| Position jump | 6 m offset to controller-facing position | `ABORTED_LANDED / TRACKING_ERROR` |
| Stale telemetry | Controller-facing position updates stop; raw PX4 subscriptions continue | `FAILED / LANDING_UNVERIFIED` |
| Geofence | Controller-facing position offset beyond its configured fence | `ABORTED_LANDED / GEOFENCE` |

These are deliberately **adapter-observation fault injections**, not Gazebo sensor-plugin corruption or PX4 estimator fault tests. Every trace retains raw PX4 position/status/land observations separately from controller-facing observations and records the fault provenance. The stale case must remain a controller failure even if an independent PX4 state sample later confirms landing and disarm; the validator rejects any unobserved, unknown, or unsafe state.

The uploaded `gazebo-x500-sitl-evidence` artifact contains:

- `evidence.json` and `summary.md` with the `GAZEBO_SITL` claim tag, UTC timestamps, outcomes, reasons, independent landed/disarmed observations and rejected/unknown states;
- `versions.txt`, `params.txt`, and the observed PX4 DDS topic list;
- per-mission ROS controller logs and 20 Hz raw/controller position and setpoint traces;
- PX4 console and Micro XRCE-DDS agent logs, plus PX4 ULogs.

`config/versions.json` pins the source refs and runner label. GitHub-hosted Ubuntu and ROS/Gazebo apt repositories are rolling rather than immutable snapshots, so exact resolved package versions are recorded per run; results from different apt snapshots should not be treated as bit-for-bit reproducible. The suite does not establish physical-flight performance, Gazebo sensor fidelity, PX4 estimator response to real GNSS faults, or certification evidence. Kinematic harness output remains separate and is not PX4 evidence.

The adapter still requires `UAV_SIMULATION_ONLY=1` to start. Do not remove that guard or run the controller on a physical-autopilot network.
