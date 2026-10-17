import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))

from sim_demo_ws import SimulationDemo, handle_message  # noqa: E402


class SimulationDemoTests(unittest.TestCase):
    def setUp(self):
        self.sim = SimulationDemo(
            lambda features: (
                "VIBRATION_PATTERN" if features[5] > 0.4 else "NORMAL",
                0.97 if features[5] > 0.4 else 0.99,
            )
        )

    def test_sample_is_labeled_synthetic_and_reports_model_advisory(self):
        sample = self.sim.tick()
        self.assertEqual(sample["source"], "SIMULATION_DEMO")
        self.assertEqual(sample["ml_scope"], "SYNTHETIC DEMO ONLY")
        self.assertEqual(sample["ml_status"], "NORMAL")
        self.assertFalse(sample["armed"])

    def test_start_and_anomaly_injection_are_confined_to_demo(self):
        accepted, _ = self.sim.command("start")
        self.assertTrue(accepted)
        self.sim.tick()
        accepted, _ = self.sim.command("inject_fault")
        self.assertTrue(accepted)
        sample = self.sim.tick()
        self.assertEqual(sample["ml_status"], "VIBRATION_PATTERN")
        self.assertEqual(sample["source"], "SIMULATION_DEMO")

    def test_manual_land_finishes_as_aborted_mission_after_descent(self):
        self.sim.command("start")
        self.sim.tick()
        self.assertTrue(self.sim.command("land")[0])
        self.assertEqual(self.sim.tick()["state"], "LANDING")
        for _ in range(12):
            sample = self.sim.tick()
        self.assertEqual(sample["state"], "ABORTED_LANDED")
        self.assertEqual(sample["outcome"], "ABORTED_LANDED")
        self.assertEqual(sample["alert"], "OPERATOR_LAND")
        self.assertFalse(sample["armed"])

    def test_abort_lands_with_aborted_outcome(self):
        self.sim.command("start")
        self.sim.tick()
        self.assertTrue(self.sim.command("abort")[0])
        for _ in range(12):
            sample = self.sim.tick()
        self.assertEqual(sample["state"], "ABORTED_LANDED")
        self.assertEqual(sample["outcome"], "ABORTED_LANDED")
        self.assertEqual(sample["alert"], "OPERATOR_ABORT")

    def test_reset_is_rejected_while_simulated_vehicle_is_armed(self):
        self.sim.command("start")
        self.assertFalse(self.sim.command("reset")[0])
        self.assertTrue(self.sim.armed)

    def test_unknown_commands_and_malformed_requests_are_rejected(self):
        self.assertFalse(self.sim.command("arm_real_drone")[0])
        ack = handle_message(self.sim, '{"type":"command","command":"arm_real_drone","request_id":"t1"}')
        self.assertFalse(ack["accepted"])
        self.assertEqual(ack["request_id"], "t1")
        self.assertFalse(handle_message(self.sim, "not-json")["accepted"])


if __name__ == "__main__":
    unittest.main()
