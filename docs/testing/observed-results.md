# Observed validation — 2026-10-15

## Environment
Ubuntu 24.04.3 LTS x86_64, GCC 13.3.0, Python 3.12.14, Node 24.19.0.
CMake 3.31.6 was installed into an isolated temporary tooling directory after the initial inspection.
ROS 2, PX4, Gazebo, Docker and GoogleTest unavailable. Browser package exists but Chromium does not;
a Chromium download failed (invalid/truncated archive). No browser screenshot or visual QA is claimed.

| Check | Command | Observed result |
|---|---|---|
| Strict C++ + undefined-behavior sanitizer | bash scripts/test_core.sh | 25 checks passed |
| CMake configure/build | cmake -S . -B build/cmake; cmake --build build/cmake | core_test and uav_demo built |
| CTest | ctest --test-dir build/cmake --output-on-failure | 1/1 test passed (contains 25 checks) |
| Harness scenarios | python3 -m unittest discover -s tests/integration -v | 5/5 passed (four harness scenarios + replay test) |
| Python syntax | python3 -m compileall scripts machine_learning | passed |
| WebSocket replay | subprocess server + async loopback client | 3 ordered source-labeled messages received |
| React/TS production build | npm run build | tsc passed; Vite built 760 modules |
| ML pipeline | python3 machine_learning/training/train.py | models and JSON evaluation generated |
| ROS adapter / live ROS bridge | colcon / actual PX4 | NOT RUN: dependencies unavailable |
| Gazebo SDF runtime | gz sdf -k / flight | NOT RUN: Gazebo unavailable |
| UI browser / screenshots | Playwright | BLOCKED: missing Chromium; failed browser archive download |
| CI hosted runner | GitHub Actions | configuration supplied; not run remotely |

No PX4 mission completion, tracking error, energy, CPU utilization, end-to-end latency, safety certification,
or hardware behavior has been measured. Harness tests validate software logic only, not autopilot integration.

## Synthetic ML measurements
4,000 training rows across 40 episodes; 2,000 test rows across 20 disjoint episodes. Seeds 41/93, shifted test wind noise.

| Model | Precision | Recall | F1 | False positive rate |
|---|---:|---:|---:|---:|
| Random Forest health | 0.9288 | 1.0000 | 0.9631 | 0.0627 |
| Isolation Forest | 0.8141 | 0.9778 | 0.8884 | 0.1827 |

Artificial fault generation makes separation easier than real telemetry; these numbers are not deployment performance.
RF batch prediction average was approximately 0.0043 ms/sample on the initial run. This excludes transport,
preprocessing, model startup and scheduling; it is not single-sample tail latency or a control-loop timing guarantee.
Route regressor MAE: 0.4387 Wh. Deterministic synthetic formula MAE: 0.1546 Wh.
The deterministic baseline **outperformed** the learned model and remains preferred.
The deterministic formula is also the synthetic generator's noiseless relation; this is an experiment, not independent physical validation.
Detailed confusion matrices, versions and limitations: [ml-report.json](ml-report.json).

## Phase 1 acceptance checklist — pending
On the target host: record source/package lock; colcon build; inspect QoS/versioned topics; verify default x500;
run ten bounded takeoff/land repetitions; retain ULogs and controller requests/ACKs/observations;
terminate heartbeat during climb and verify configured PX4 Land fallback; confirm no arm on stale input;
only then mark Phase 1 accepted. No acceptance item is satisfied by the kinematic harness.
