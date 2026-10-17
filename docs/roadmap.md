# Incremental roadmap and acceptance gates

| Phase | Delivered now | Still required | Gate |
|---|---|---|---|
| 1 | launch scripts, ROS control source, telemetry readers | target install, compile, real SITL run, ULog evidence | repeated takeoff/land and verified telemetry |
| 2 | shared core/ROS mission engine, waypoint configuration, ROS abort service source, GoogleTest suite, ament core package | target-host build/flight, validated RTL, ROS integration tests | actual SITL waypoint + abort |
| 3 | synthetic episode generator, Isolation Forest, RF health, route regression, report, optional exporter | PX4 datasets, calibration, ONNX parity, C++ worker/watchdog | measured control jitter unaffected by inference |
| 4 | architecture/contracts only | Gazebo cameras/depth, OpenCV/PyTorch detector, ground-truth comparison | obstacle precision/recall and depth error |
| 5 | built React monitor, replay/live bridge sources, trace/export | tested live PX4 integration, command gateway, Three.js, full recording/replay seek, waypoint editor, Tailwind | UI accurately matches real PX4 states |
| 6 | six kinematic regression cases plus replay validation | wind/GPS/battery/actuator/comms fault scenarios, resource/jitter reports | all critical SITL failsafes with observed logs |

Urban, weather/disturbance and health Gazebo worlds remain future work; no empty world is presented as implemented.
Model binaries and datasets stay out of Git. Synthetic artifacts are reproducible, not evidence of physical performance.
Next action: execute Phase 1 on the documented target, record exact packages/source commits, resolve build/message/QoS issues,
and retain logs before increasing autonomy. Multi-vehicle namespaces, target IDs and resource admission are later work.
