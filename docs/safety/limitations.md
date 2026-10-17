# Safety scope and unresolved issues
Simulation-only research code; no real aircraft support, firmware flashing, serial links or live brokerage-style autonomy.
Do not interpret passing harness/ML tests as proof of aerodynamic stability or physical safety.

Implemented: finite position checks, receipt freshness, minimum battery, radial local fence, altitude envelope,
bounded target increments, explicit request/ACK/observed state distinction, command retry limits, mission deadlines.
The ROS adapter uses these checks but has not been built/run with ROS/PX4 here.

Not implemented: raw GPS validity, velocity/acceleration envelope, estimator resets, missing heartbeat watchdog independent of process,
source timestamp replay protection, runtime ML timeout/uncertainty checks, obstacle clearance, authenticated simulator identity,
UI command authorization, position-aware RTL, fault recovery state machine, physical motor model.
PX4's own configured heartbeat loss behavior is independent of the C++ process; it still requires real SITL fault testing.
A finite position inside the fence does not imply a collision-free path. Battery classifier predictions are suspected patterns, not confirmed failures.
The environment variable guard is a friction mechanism, not a security boundary. Only run inside the isolated SITL host.
