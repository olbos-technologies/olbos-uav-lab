# Fault-injection scenarios

`bash scripts/run_scenarios.sh` builds `uav_scenarios`, runs every scenario in
`core/include/uav/simulation.hpp` (`catalog()`), and writes to `artifacts/scenarios/`:

- `safety-report.json`: machine-readable results; the process exits non-zero if any scenario fails;
- `safety-report.md` / `safety-report.html`: rendered by `scripts/safety_report.py` (CI adds the Markdown to the job summary);
- `traces/<name>.jsonl`: per-tick telemetry in the dashboard replay format
  (`python3 scripts/telemetry_ws.py artifacts/scenarios/traces/motor_failure.jsonl`).

Run a subset with `bash scripts/run_scenarios.sh gps_jump motor_failure`. The same catalog runs as GoogleTest
(`scenario_test`, one test per scenario), plus a control test proving that a disabled tracking monitor makes the suite fail.

## Plant model
A deterministic 20 Hz kinematic approximation of a PX4 multicopter: first-order position tracking (0.3 s),
5 m/s horizontal and 3 m/s vertical limits, 90% wind rejection up to 8 m/s with any excess pushing the airframe,
0.7 m/s Land descent, position hold when setpoints stop, and auto-disarm 1 s after touchdown once the land detector agrees.
It is not aerodynamics, PX4 SITL or flight evidence. The same faults still need PX4 SITL runs with ULog evidence (roadmap phase 6).

## Invariants checked on every scenario
| Check | Meaning |
|---|---|
| command_envelope | every streamed setpoint is finite, inside the fence radius and the altitude band |
| setpoint_rate | consecutive streamed setpoints never move faster than `max_speed` |
| arm_preconditions | arm is only requested with valid, fresh position and sufficient battery |
| stop_streaming_on_fault | no offboard setpoints once any fault reason is set |
| terminal_outcome | the mission ends within deadline + landing timeout |
| expected_outcome / expected_alert | outcome and first alert match the scenario |
| detection_latency | first alert arrives within the scenario's limit after the fault becomes observable |
| honest_landing_claim | COMPLETE / ABORTED_LANDED only when the simulated vehicle is on the ground and disarmed |
| complete_only_when_clean | nominal completion is never reported after a fault |
| land_retried | (land_ignored) Land keeps being requested while the autopilot ignores it |

Physical warnings (drift beyond the fence, impact touchdown, vehicle left airborne) are reported separately: they describe
the vehicle, not a controller defect, and do not fail the suite.

## Catalog
| Scenario | Fault | Expected |
|---|---|---|
| nominal | none | COMPLETE, no alert |
| wind_light | steady 4 m/s crosswind | COMPLETE, no alert |
| wind_strong | 14 m/s wind at 8 s | TRACKING_ERROR within 4 s, ABORTED_LANDED |
| battery_sag | battery drops to 12% at 8 s | LOW_BATTERY immediately |
| battery_fast_drain | 4%/s drain | LOW_BATTERY when the 20% reserve is crossed |
| gps_loss | position invalid at 8 s | INVALID_POSITION immediately |
| gps_jump | estimate jumps 6 m at 8 s | TRACKING_ERROR within 3 s |
| estimator_reset_outside_fence | estimate jumps 45 m at 8 s | GEOFENCE immediately |
| motor_failure | thrust loss at 8 s | TRACKING_ERROR within 3 s; impact is flagged |
| telemetry_freeze | all telemetry frozen at 8 s | STALE_TELEMETRY; FAILED because landing cannot be verified |
| telemetry_jitter | receipt age up to 0.75 s | COMPLETE, no false alarm |
| control_loop_stall | controller unscheduled 0.8 s | CONTROL_LOOP_STALL |
| operator_abort | abort at 8 s | OPERATOR_ABORT |
| arm_rejected | arm rejected | COMMAND_REJECTED, never armed |
| offboard_ignored | mode request never answered | COMMAND_TIMEOUT, never armed |
| land_ignored | battery fault, Land ignored | Land retried until timeout; FAILED, no landing claim |
| land_detector_stuck | touchdown never reported | LANDING_UNVERIFIED, never COMPLETE |

To add a scenario, append it to `catalog()` with its `Faults` and `Expectation`; it automatically joins the CLI, CI and GoogleTest.
