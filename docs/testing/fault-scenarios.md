# Fault-injection scenarios

`bash scripts/run_scenarios.sh` builds `uav_scenarios`, runs every scenario in
`core/include/uav/simulation.hpp` (`catalog()`) plus the envelope sweeps (`sweeps()`), and writes to `artifacts/scenarios/`:

- `safety-report.json`: machine-readable results; the process exits non-zero if any scenario fails or any sweep
  point breaks a safety invariant (exit 2 for an invalid configuration);
- `safety-report.md` / `safety-report.html`: rendered by `scripts/safety_report.py` (CI adds the Markdown to the job summary);
- `traces/<name>.jsonl`: per-tick telemetry in the dashboard replay format
  (`python3 scripts/telemetry_ws.py artifacts/scenarios/traces/motor_failure.jsonl`).

Run a subset with `bash scripts/run_scenarios.sh gps_jump motor_failure` (sweeps are skipped when scenarios are named;
`--no-sweeps` skips them explicitly). The same catalog runs as GoogleTest (`scenario_test`, one test per scenario), plus a
control test proving that a disabled tracking monitor makes the suite fail.

## Testing your own mission
`--config` takes the same ROS parameter file the PX4 adapter node loads, so the suite judges exactly the mission you fly:

```bash
bash scripts/run_scenarios.sh --config config/waypoint-mission.yaml --out artifacts/scenarios/waypoint-mission
```

Parsing and the parameter-to-mission translation live in `core/include/uav/params.hpp` and are shared with the node, so
defaults cannot drift apart. Unknown keys and malformed numbers are rejected rather than ignored. The catalog adapts to the
mission: a fault-free probe run measures the airborne window and faults are injected at its midpoint; detection limits
derive from the configured tracking monitor and stale threshold; GPS-jump size and the estimator reset scale with the
tracking limit and fence. CI runs the built-in route and every `config/*.yaml`.

## Envelope sweeps
Each sweep repeats one fault across a range and records where the mission stops completing, how the controller stops it,
and whether every safety invariant still holds (a broken invariant fails the run). The report charts maximum distance from
home against fault size, with a table view.

| Sweep | Range | Built-in route result |
|---|---|---|
| steady wind | 0–20 m/s | completes up to 6 m/s (8 m/s airspeed minus 2 m/s mission speed); above, LEG_TIMEOUT/TRACKING_ERROR; drift reaches 52 m at 20 m/s |
| position jump | 0–10 m | up to 7.5 m completes **undetected** with the airframe displaced by the jump; larger jumps trip TRACKING_ERROR while the autopilot catches up |
| telemetry gap | 0–3 s | completes up to 0.75 s; from the 1 s stale limit, STALE_TELEMETRY |
| control-loop stall | 0–1.5 s | completes up to 0.4 s; from 0.5 s, CONTROL_LOOP_STALL |

## Plant model
A deterministic 20 Hz kinematic approximation of a PX4 multicopter: first-order tracking (0.3 s) of the *estimated*
position onto the setpoint, as PX4 does, so an estimate bias moves the real airframe; 5 m/s horizontal and 3 m/s vertical
command limits; an assumed 8 m/s airspeed authority, with ground velocity = air velocity + wind (wind within authority is
fully cancelled, as PX4's integrator would; beyond it the airframe is pushed off course); 0.7 m/s Land descent, position
hold when setpoints stop, and auto-disarm 1 s after touchdown once the land detector agrees. The airspeed authority is a
property of the airframe, not the controller: the wind envelope moves with it.
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

Physical warnings (drift beyond the fence, impact touchdown, vehicle left airborne, a mission reported COMPLETE while the
airframe was more than 1 m from its estimated position) are reported separately: they describe the vehicle or limits of
mission-level monitoring, not a controller defect, and do not fail the suite.

## Catalog
Times below are for the built-in square route, where the probe places faults at 12 s; other missions use their own midpoint.

| Scenario | Fault | Expected |
|---|---|---|
| nominal | none | COMPLETE, no alert |
| wind_light | steady 4 m/s crosswind | COMPLETE, no alert |
| wind_strong | 14 m/s wind mid-flight | TRACKING_ERROR, ABORTED_LANDED; drift beyond the fence flagged |
| battery_sag | battery drops below the reserve mid-flight | LOW_BATTERY immediately |
| battery_fast_drain | drain that crosses the reserve mid-flight | LOW_BATTERY when the reserve is crossed |
| gps_loss | position invalid mid-flight | INVALID_POSITION immediately |
| gps_jump | estimate jumps 2 × tracking limit | not detectable at mission level: COMPLETE (or TRACKING_ERROR); displacement flagged |
| estimator_reset_outside_fence | estimate jumps fence + 5 m | GEOFENCE immediately |
| motor_failure | thrust loss mid-flight | TRACKING_ERROR (or UNEXPECTED_VEHICLE_STATE after a low-altitude impact); impact flagged |
| telemetry_freeze | all telemetry frozen mid-flight | STALE_TELEMETRY; FAILED because landing cannot be verified |
| telemetry_jitter | receipt age up to 70% of the stale limit | COMPLETE, no false alarm |
| control_loop_stall | controller unscheduled 0.8 s mid-flight | CONTROL_LOOP_STALL |
| operator_abort | abort mid-flight | OPERATOR_ABORT |
| arm_rejected | arm rejected | COMMAND_REJECTED, never armed |
| offboard_ignored | mode request never answered | COMMAND_TIMEOUT, never armed |
| land_ignored | battery fault, Land ignored | Land retried until timeout; FAILED, no landing claim |
| land_detector_stuck | touchdown never reported | LANDING_UNVERIFIED, never COMPLETE |

To add a scenario, append it to `catalog()` with its `Faults` and `Expectation`; it automatically joins the CLI, CI and GoogleTest.
