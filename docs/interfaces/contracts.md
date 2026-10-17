# Interfaces

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
