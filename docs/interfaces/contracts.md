# Olbos UAV Lab interface contracts

| Producer → Consumer | Contract | Units / frame | Failure treatment |
|---|---|---|---|
| VehicleLocalPosition → core | position, xy_valid, z_valid, receipt time | PX4 NED m → ROS ENU m | invalid/stale → latch Fault |
| BatteryStatus → core | connected, remaining, receipt time | fraction [0,1] | unknown/out of range → Fault |
| VehicleStatus → controller | arming_state, nav_state | PX4 enums | state changes must be observed |
| VehicleLandDetected → controller | landed, receipt time | boolean | no completion from stale sample |
| Controller → PX4 | OffboardControlMode + TrajectorySetpoint | NED; microsecond ROS timestamps | 20Hz heartbeat; independent PX4 loss action |
| Controller → PX4 | VehicleCommand | target system=1 component=1 | three attempts / state verification |
| PX4 → controller | VehicleCommandAck | command, result | accepted/in progress logged; rejection Fault |
| Bridge → UI | JSON snapshot | ENU m, battery fraction, time_s | stale after 2s; source always visible |
| ML → planner (future) | score, model version, evidence, age, optional uncertainty | advisory | unavailable/timeout → deterministic baseline |

ROS 2 topic names append `_vN` when a message declares MESSAGE_VERSION >0. The adapter derives this from the compiled message type.
DDS QoS for PX4 outputs is best-effort sensor-data depth 5. Inputs use depth 10.
Use matching firmware message definitions; the source sync script records selected commits.
Current freshness is local steady-clock receipt age. Delayed/replayed DDS packets, EKF reset, and source timestamp validation
are unresolved and must be addressed before expanding autonomous missions.

Replay JSON example:
```json
{"source":"KINEMATIC_HARNESS","transport":"REPLAY","time_s":4.0,"east":0,"north":0,"altitude":5,"battery":0.9,"mode":"NAVIGATION","alert":"NORMAL"}
```
Live bridge uses `PX4_SITL_UNVERIFIED`: it cannot attest hardware identity. Nav mode is the raw PX4 enum;
alert UNASSESSED means no C++ supervisor output is wired into this bridge yet. No artificial ML confidence is shown.
Dashboard controls are disabled until a command gateway with IDs, expiry, allow-list, acknowledgments, and confirmed execution exists.

## Shared control contract (0.2)
`MissionController` owns the same state machine for the ROS adapter and harness. It consumes a local
elapsed monotonic timestamp, a consolidated Telemetry receipt age and explicit armed/offboard/landing/landed flags.
Adapter startup must define elapsed time zero; the mission deadline is measured from that origin.
The next tick after `request_abort()` stops offboard streaming and requests Land if the mission has observed arming.
The service `/sitl_mission/abort` uses std_srvs/srv/Trigger. Its `success=true` means cancellation was queued,
not that PX4 accepted Land or that landing completed. PX4 ACKs are filtered by destination system 1/component 191
and the pending request type; request-attempt-level correlation is not implemented.

`waypoints_enu` is a flattened list of east/north/up triples following the vertical takeoff waypoint.
Empty means ascent/land smoke test; at most 63 extra waypoints are accepted. Each point must satisfy the deterministic
geofence and altitude checks before construction. No learned route can replace this plan in 0.2.
