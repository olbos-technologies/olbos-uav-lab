# Safety scope and unresolved issues
Simulation-only research code; no real aircraft support, firmware flashing, serial links or live brokerage-style autonomy.
Do not interpret passing harness/ML tests as proof of aerodynamic stability or physical safety.

Implemented: finite position checks, receipt freshness, minimum battery, radial local fence, altitude envelope,
bounded target increments, explicit request/ACK/observed state distinction, command retry limits, mission deadlines,
setpoint tracking-error monitor (default 3 m for 1 s; tuned only against the kinematic plant, not SITL takeoff lag).
The ROS adapter now uses the same tested MissionController as the harness, but has not been built/run with ROS/PX4 here.

Not implemented: raw GPS validity, velocity/acceleration envelope, estimator resets, missing heartbeat watchdog independent of process,
source timestamp replay protection, runtime ML timeout/uncertainty checks, obstacle clearance, authenticated simulator identity,
UI command authorization, position-aware RTL, automatic fault recovery/restart, physical motor model.
PX4's own configured heartbeat loss behavior is independent of the C++ process; it still requires real SITL fault testing.
A finite position inside the fence does not imply a collision-free path. Battery classifier predictions are suspected patterns, not confirmed failures.
The environment variable guard is a friction mechanism, not a security boundary. Only run inside the isolated SITL host.

In 0.2 operator abort is latched, command retries/deadlines are bounded, and fresh landed/disarmed state is required
for any verified landing outcome. A stale-data descent remains unverified. The core's high-level FSM supports abort/emergency/
communication-loss transitions; position-aware RTL and recovery are still unimplemented.

## Fault-injection findings (kinematic suite)
`bash scripts/run_scenarios.sh` exercises 17 faults; see docs/testing/fault-scenarios.md. The controller logic passes all of them,
but three physical limits remain and are flagged in every report:
- Wind beyond control authority keeps pushing the airframe during the Land descent. In the 14 m/s case the vehicle ends
  about 64 m from home despite detection within 4 s. Mitigation is operational: a pre-flight wind limit, not software.
- From 5 m altitude a total thrust loss reaches the ground before any supervisory monitor can respond. Detection only
  stops further commands; PX4 failure detection, parachutes or redundancy own this hazard.
- If the autopilot ignores Land, the controller keeps requesting it until landing_timeout and then reports FAILED with the
  vehicle still airborne. PX4's own failsafe configuration must cover this case.
Before the tracking monitor was added, the same suite showed strong wind undetected for 17.5 s (89 m drift), a 6 m GPS jump
undetected for 27 s and motor failure noticed only after impact.
