# Olbos UAV Lab

Autonomous UAV simulation and machine-learning research platform built with C++20, PX4, ROS 2, and Gazebo. MIT licensed.
**Version 0.1 — tested standalone foundation; PX4 SITL acceptance pending.**

Dashboard source and production build are included. Screenshot/browser verification is pending: Chromium was unavailable and its download failed in the initial environment.

## What runs now
- C++20 core: explicit ENU/NED types, deterministic waypoint stepping, mission transition validation, independent safety checks.
- Kinematic reference harness with nominal, low-battery, stale-telemetry and geofence regression cases.
- React/TypeScript/Vite/Recharts read-only dashboard with trajectory, altitude history, source labels, stale detection and JSONL export.
- Read-only WebSocket replay, plus ROS 2 live telemetry bridge source.
- Offline synthetic health detection and route energy experiments with measured evaluation reports.

## PX4 integration source
ROS 2 C++ node implements heartbeat warmup, mode/arming requests, command acknowledgments, bounded ascent,
landing and completion from observed landed/disarmed state. Flight limits are ROS parameters.
Launch/runbook targets Ubuntu 22.04 + ROS 2 Humble + PX4 v1.16.0 + Gazebo Harmonic.
This adapter has **not been compiled or flown in SITL in the initial environment**. ROS/PX4/Gazebo are absent.
It must pass target-host acceptance before being called a working PX4 simulation MVP.

```bash
bash scripts/test_core.sh
bash scripts/run_demo.sh
python3 -m unittest discover -s tests/integration -v
```
Full execution commands, expected output and blockers: [installation](docs/installation/setup.md).

## Engineering documentation
- [Architecture and diagram](docs/architecture/architecture.md)
- [Compatibility matrix and official sources](docs/installation/compatibility.md)
- [Interface contracts](docs/interfaces/contracts.md)
- [Safety scope and limitations](docs/safety/limitations.md)
- [Observed validation results](docs/testing/observed-results.md)
- [Phase roadmap and unfinished deliverables](docs/roadmap.md)

No real-aircraft operation, safety certification, guaranteed ML reliability or production readiness is claimed.
ML never publishes flight commands. The dashboard command interface is deliberately disabled until an acknowledged command gateway exists.
Generated datasets, trained models, build outputs, node_modules and external checkouts are ignored.
Dependencies are partially pinned; target OS package resolution remains to be captured. Repository: olbos-uav-lab (https://github.com/olbos-technologies/olbos-uav-lab)
