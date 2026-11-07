import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts" / "sitl"))
import gazebo_evidence


class GazeboEvidenceTests(unittest.TestCase):
    def test_controller_log_preserves_raw_and_faulted_observations(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "controller.log"
            log.write_text(
                "\n".join(
                    [
                        "INJECTED_FAULT scenario=gnss_loss source=ROS_ADAPTER_OBSERVATION raw_px4_untouched=true",
                        "REQUEST command=21 (await ACK and observed state)",
                        "STATE LANDING outcome=2 reason=INVALID_POSITION waypoint=0",
                        "EVIDENCE tag=GAZEBO_SITL outcome=ABORTED_LANDED reason=INVALID_POSITION "
                        "fault=gnss_loss fault_injected=1 px4_armed=0 px4_landed=1 "
                        "px4_position_valid=1 controller_position_valid=0",
                    ]
                ),
                encoding="utf-8",
            )
            result = gazebo_evidence.parse_controller_log(log, "gnss_loss")

        self.assertEqual(result["outcome"], "ABORTED_LANDED")
        self.assertTrue(result["land_request_observed"])
        self.assertEqual(result["fault_marker"]["source"], "ROS_ADAPTER_OBSERVATION")
        self.assertEqual(result["terminal_evidence"]["px4_position_valid"], "1")
        self.assertEqual(result["terminal_evidence"]["controller_position_valid"], "0")

    def test_valid_fault_observation_and_safe_landing_pass_invariants(self):
        run = {
            "mission": "gnss_loss",
            "repeat": 1,
            "outcome": "ABORTED_LANDED",
            "reason": "INVALID_POSITION",
            "land_request_observed": True,
            "actual_landed_disarmed": True,
            "recorded_at_utc": "2026-11-07T00:00:00+00:00",
            "trace_file": "gnss_loss-1.jsonl",
            "trace_samples": 20,
            "fault_seen_in_trace": True,
            "streaming_after_fault": False,
            "terminal_evidence": {
                "tag": "GAZEBO_SITL",
                "px4_armed": "0",
                "px4_landed": "1",
                "px4_position_valid": "1",
                "controller_position_valid": "0",
                "px4_position_east": "0.000",
                "controller_position_east": "0.000",
                "px4_telemetry_age": "0.050",
                "controller_telemetry_age": "0.050",
                "fault": "gnss_loss",
                "fault_injected": "1",
            },
            "fault_marker": {
                "source": "ROS_ADAPTER_OBSERVATION",
                "raw_px4_untouched": "true",
            },
        }
        errors = []

        gazebo_evidence.validate_run(
            run, gazebo_evidence.EXPECTED[("gnss_loss", 1)], errors
        )

        self.assertEqual(errors, [])

    def test_stale_telemetry_never_passes_without_actual_safe_vehicle_state(self):
        run = {
            "mission": "stale_telemetry",
            "repeat": 1,
            "outcome": "FAILED",
            "reason": "LANDING_UNVERIFIED",
            "actual_landed_disarmed": False,
            "terminal_evidence": {
                "tag": "GAZEBO_SITL",
                "px4_armed": "0",
                "px4_landed": "1",
                "px4_position_valid": "1",
                "controller_position_valid": "1",
                "fault": "stale_telemetry",
                "fault_injected": "1",
            },
            "fault_marker": {
                "source": "ROS_ADAPTER_OBSERVATION",
                "raw_px4_untouched": "true",
            },
        }
        errors = []

        gazebo_evidence.validate_run(
            run, gazebo_evidence.EXPECTED[("stale_telemetry", 1)], errors
        )

        self.assertTrue(any("did not confirm disarmed and landed" in error for error in errors))

    def test_unknown_controller_terminal_state_is_not_a_pass(self):
        run = {
            "mission": "smoke",
            "repeat": 1,
            "outcome": "UNKNOWN",
            "reason": "UNKNOWN",
            "actual_landed_disarmed": True,
            "terminal_evidence": {},
            "fault_marker": {},
        }
        errors = []

        gazebo_evidence.validate_run(run, gazebo_evidence.EXPECTED[("smoke", 1)], errors)

        self.assertTrue(any("expected COMPLETE/NORMAL" in error for error in errors))

    def test_complete_repeated_mission_matrix_passes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "runs").mkdir()
            (root / "params.txt").write_text(
                "\n".join(
                    [
                        "configured UXRCE_DDS_DOM_ID=42",
                        "configured COM_OF_LOSS_T=1",
                        "configured COM_OBL_RC_ACT=4",
                        "configured COM_RC_IN_MODE=4",
                    ]
                ),
                encoding="utf-8",
            )
            (root / "params-readback.txt").write_text(
                "UXRCE_DDS_DOM_ID: 42\nCOM_OF_LOSS_T: 1.0\n"
                "COM_OBL_RC_ACT: 4\nCOM_RC_IN_MODE: 4\n",
                encoding="utf-8",
            )
            (root / "versions.txt").write_text(
                "claim_tag=GAZEBO_SITL\nsimulator=gz_x500\n"
                "px4_ref=v1.16.0\nagent_ref=v2.4.3\n",
                encoding="utf-8",
            )
            for (mission, repeat), (outcome, reason) in gazebo_evidence.EXPECTED.items():
                fault = mission in gazebo_evidence.FAULT_REASONS
                offset = 6.0 if mission == "position_jump" else 45.0 if mission == "geofence" else 0
                evidence = {
                    "tag": "GAZEBO_SITL",
                    "px4_armed": "0",
                    "px4_landed": "1",
                    "px4_position_valid": "1",
                    "controller_position_valid": "0" if mission == "gnss_loss" else "1",
                    "px4_position_east": "0.000",
                    "controller_position_east": f"{offset:.3f}",
                    "px4_telemetry_age": "0.050",
                    "controller_telemetry_age": "3.000" if mission == "stale_telemetry" else "0.050",
                    "fault": mission if fault else "none",
                    "fault_injected": "1" if fault else "0",
                }
                record = {
                    "mission": mission,
                    "repeat": repeat,
                    "recorded_at_utc": "2026-11-07T00:00:00+00:00",
                    "outcome": outcome,
                    "reason": reason,
                    "land_request_observed": True,
                    "actual_landed_disarmed": True,
                    "vehicle_status_arming_state": 1,
                    "vehicle_land_detected": True,
                    "abort_service_accepted": mission == "abort",
                    "trace_file": f"{mission}-{repeat}.jsonl",
                    "trace_samples": 20,
                    "fault_seen_in_trace": fault,
                    "streaming_after_fault": False,
                    "terminal_evidence": evidence,
                    "fault_marker": (
                        {
                            "scenario": mission,
                            "source": "ROS_ADAPTER_OBSERVATION",
                            "raw_px4_untouched": "true",
                        }
                        if fault
                        else {}
                    ),
                }
                (root / "runs" / f"{mission}-{repeat}.json").write_text(
                    json.dumps(record), encoding="utf-8"
                )

            args = type(
                "Args",
                (),
                {
                    "input": str(root),
                    "output": str(root / "evidence.json"),
                    "summary": str(root / "summary.md"),
                },
            )()
            result = gazebo_evidence.validate(args)

            self.assertEqual(result, 0)
            self.assertIn("GAZEBO_SITL", (root / "summary.md").read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
