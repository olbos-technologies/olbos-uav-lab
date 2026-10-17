# Olbos UAV Lab

Autonomous UAV simulation and machine-learning research platform built with C++20, PX4, ROS 2, and Gazebo. MIT licensed.
**Version 0.2 — shared C++ mission architecture; PX4 SITL acceptance pending.**

![Dashboard replaying a completed kinematic-harness mission](docs/dashboard.png)

The dashboard was checked in headless Chromium against harness replays (nominal and low-battery abort) in dark, light and phone layouts.

## What runs now
- Modular C++20 core: explicit ENU/NED types, independent safety supervisor, validated waypoint plans and a shared deterministic MissionController.
- Kinematic harness using the same mission engine as ROS: nominal waypoints, low battery, stale telemetry, geofence, operator abort and command rejection.
- Fault-injection safety suite: 17 scenarios (wind, battery, GPS loss/jump, estimator reset, motor failure, telemetry freeze/jitter, control-loop stall, abort, rejected/ignored commands, stuck land detector) checked by independent invariants, plus envelope sweeps (wind, position jump, telemetry gap, loop stall) that chart where a mission stops completing. Runs against any node parameter file (`--config`), gated in CI and summarised as an HTML/Markdown safety report. See [fault scenarios](docs/testing/fault-scenarios.md).
- React/TypeScript/Vite/Recharts read-only dashboard: mission phase timeline, waypoint progress, auto-zoom local trajectory or a geographic map view (local ENU telemetry drawn around a display origin; not GPS), altitude/battery charts, event log, stale detection, auto-reconnect, light/dark themes, JSONL export and in-browser replay of any trace file (**Open trace** or drag and drop, e.g. `artifacts/scenarios/traces/motor_failure.jsonl`).
- Read-only WebSocket replay, plus ROS 2 live telemetry bridge source.
- Offline synthetic health detection and route energy experiments with measured evaluation reports.

## Geographic map display
The trajectory panel's **Map** toggle draws local telemetry over OpenStreetMap tiles around an editable display origin (default: a generic Tulsa map center; your choice is remembered in the browser). The default **Local** view needs no network access; tiles are only requested while Map is selected. This visualizes local east/north offsets; it is **not GPS**, a real position fix, or a navigation aid. The map remains read-only. Configure another compatible XYZ raster tile endpoint with `VITE_MAP_TILE_URL`; see `dashboard/.env.example`. The OpenStreetMap Foundation tile service is best-effort and used here for low-volume development; use a dedicated provider for a public or production service, retain visible attribution, and follow the provider's terms.

## Local dashboard demo: advisory ML and controls
The dashboard can connect to a separate loopback-only demonstration service. It publishes a deterministic synthetic route, runs the synthetic Random Forest created by `machine_learning/training/train.py`, and accepts `Start`, `Land`, `Abort`, `Test anomaly`, and `Reset` commands for that in-memory demo only. The displayed model result is an advisory from generated sensor features; it is not validated against flight data and never controls the simulation or a vehicle.

Install the pinned Python requirements and build the synthetic model once:

```bash
python3 -m pip install -r machine_learning/requirements.txt
python3 machine_learning/training/train.py
```

Start the demo bridge in one terminal:

```bash
bash scripts/run_sim_demo.sh
```

In another terminal, start the dashboard and open the loopback URL:

```bash
cd dashboard
npm ci
npm run dev
```

Open `http://127.0.0.1:5173/?ws=ws%3A%2F%2F127.0.0.1%3A8766`. The **Safety & health** panel reports the synthetic model label and top-class score. Start a mission, then choose **Test anomaly** to send simulated vibration features through the model. **Land** and **Abort** change only the in-memory demo state and end with an aborted-mission outcome. The bridge binds to `127.0.0.1:8766`, accepts the dashboard's local development origin, and has no PX4, ROS, networked aircraft, or real-command path. The standard telemetry bridge on port 8765 remains read-only.

Run the bridge unit tests with `python3 -m unittest discover -s tests/integration -p 'test_sim_demo.py'` (after installing the pinned Python requirements for the WebSocket package).

## PX4 integration source
ROS 2 C++ node adapts PX4 messages to the shared core exported by the uav_core ament package. It supports heartbeat warmup, bounded requests, waypoint parameters, an abort service and completion from fresh landed/disarmed observations. Command acceptance never substitutes for observed state.
Launch/runbook targets Ubuntu 22.04 + ROS 2 Humble + PX4 v1.16.0 + Gazebo Harmonic.
An automated PX4 SITL acceptance job (`.github/workflows/sitl.yml`, PX4's headless SIH quadrotor) flies smoke, waypoint and abort missions on CI and keeps ULogs as evidence; see [installation](docs/installation/setup.md#automated-sitl-acceptance-headless-ci). Gazebo runs remain manual.
On CI it compiles, connects to PX4 v1.16 through the Micro XRCE-DDS agent and passes smoke, square-waypoint and operator-abort missions on PX4's SIH quadrotor, with landings confirmed by PX4's own land detector. Gazebo (`gz_x500`) runs and real-airframe tuning of the tracking monitor remain manual.

```bash
bash scripts/test_core.sh
bash scripts/run_demo.sh
bash scripts/run_scenarios.sh   # writes artifacts/scenarios/safety-report.html
bash scripts/run_scenarios.sh --config config/waypoint-mission.yaml --out artifacts/scenarios/waypoint-mission
python3 -m unittest discover -s tests/integration -v
```
Full execution commands, expected output and blockers: [installation](docs/installation/setup.md).

## Mission architecture and testing
The ROS adapter and kinematic plant both consume `MissionController::tick()` decisions. Safety checks precede
active flight decisions; cancellation is latched, and stale landing observations cannot produce successful completion.
The default ROS configuration remains an ascent/land smoke test; `config/waypoint-mission.yaml` adds a short waypoint route.

```bash
# With libgtest-dev installed:
cmake -S . -B build -DUAV_BUILD_GTEST=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

See the [0.2 observed results](docs/testing/v0.2-results.md) for actual checks and remaining blockers.

## Engineering documentation
- [Architecture and diagram](docs/architecture/architecture.md)
- [Compatibility matrix and official sources](docs/installation/compatibility.md)
- [Interface contracts](docs/interfaces/contracts.md)
- [Safety scope and limitations](docs/safety/limitations.md)
- [Observed validation results](docs/testing/observed-results.md)
- [Phase roadmap and unfinished deliverables](docs/roadmap.md)

No real-aircraft operation, safety certification, guaranteed ML reliability or production readiness is claimed.
ML never publishes flight commands. The command buttons are enabled only for the explicitly labeled local demo source; a production/PX4 command gateway is not implemented.
Generated datasets, trained models, build outputs, node_modules and external checkouts are ignored.
Dependencies are partially pinned; target OS package resolution remains to be captured. Repository: olbos-uav-lab (https://github.com/olbos-technologies/olbos-uav-lab)
