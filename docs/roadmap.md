# Incremental roadmap and acceptance gates

| Phase | Delivered now | Still required | Gate |
|---|---|---|---|
| 1 | launch scripts, ROS control source, telemetry readers; automated PX4 v1.16 SITL job (SIH quadrotor) builds, connects and passes takeoff/land with ULog evidence on every flight-path change | Gazebo `gz_x500` run on a target host | repeated takeoff/land and verified telemetry (met on SIH in CI) |
| 2 | shared core/ROS mission engine, waypoint configuration, ROS abort service, GoogleTest suite, ament core package; square waypoint and operator abort pass in PX4 SITL (SIH) | validated RTL, the 17 fault scenarios in SITL, Gazebo runs | actual SITL waypoint + abort (met on SIH in CI) |
| 3 | synthetic episode generator, Isolation Forest, RF health, route regression, report, optional exporter | PX4 datasets, calibration, ONNX parity, C++ worker/watchdog | measured control jitter unaffected by inference |
| 4 | architecture/contracts only | Gazebo cameras/depth, OpenCV/PyTorch detector, ground-truth comparison | obstacle precision/recall and depth error |
| 5 | built React monitor, replay/live bridge sources, trace/export, origin-based map view for local ENU telemetry | tested live PX4 integration, GPS-backed map mode, command gateway, Three.js, full recording/replay seek, waypoint editor, Tailwind | UI accurately matches real PX4 states |
| 6 | 17-scenario kinematic fault-injection suite (wind, battery, GPS loss/jump, estimator reset, motor failure, telemetry freeze/jitter, loop stall, abort, rejected/ignored commands, stuck land detector) with independent invariants, CI gate and HTML/Markdown safety report; tracking-error monitor | the same faults in PX4 SITL with ULog evidence, resource/jitter reports | all critical SITL failsafes with observed logs |

Urban, weather/disturbance and health Gazebo worlds remain future work; no empty world is presented as implemented.
Model binaries and datasets stay out of Git. Synthetic artifacts are reproducible, not evidence of physical performance.
Next action: execute Phase 1 on the documented target, record exact packages/source commits, resolve build/message/QoS issues,
and retain logs before increasing autonomy. Multi-vehicle namespaces, target IDs and resource admission are later work.
