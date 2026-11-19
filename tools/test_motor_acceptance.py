import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import motor_acceptance
from test_stlink_bench import SimulatedBench


class AcceptanceTest(unittest.TestCase):
    def records(self, operation="velocity", position=0, velocity=0.1):
        return [dict(operation=operation, position=position, velocity=velocity, target=0.1,
                     seconds=(n + 1) * 0.1, iq=0.05, id=0, fault=0, status=0x237) for n in range(30)]

    def test_velocity_requires_tracking_and_rejects_stationary_motor(self):
        self.assertTrue(motor_acceptance.summarize(self.records())["passed"])
        self.assertFalse(motor_acceptance.summarize(self.records(velocity=0))["passed"])

    def test_position_requires_settled_target(self):
        self.assertTrue(motor_acceptance.summarize(self.records("position", 0.1, 0))["passed"])
        self.assertFalse(motor_acceptance.summarize(self.records("position", 0, 0))["passed"])
        self.assertFalse(motor_acceptance.summarize(self.records("mit", 0.1, 1))["passed"])

    def test_sparse_and_nonfinite_data_not_accepted(self):
        with self.assertRaises(ValueError):
            motor_acceptance.summarize(self.records()[:4])
        with self.assertRaises(ValueError):
            motor_acceptance.summarize(self.records(velocity=float("nan")))

    def test_preflight_failure_is_not_reported_as_motion_test(self):
        bench = SimulatedBench()
        state = dict(bench.snapshot(), offsets_valid=0, zero_reason=1, fault=0x5210)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "report.json"
            with patch.object(motor_acceptance, "Bench", return_value=bench), \
                 patch.object(bench, "snapshot", return_value=state), \
                 patch.object(motor_acceptance, "run_motion") as motion:
                with self.assertRaisesRegex(RuntimeError, "Current-zero"):
                    motor_acceptance.main(["--output", str(path)])
                motion.assert_not_called()
            report = json.loads(path.read_text())
            self.assertFalse(report["completed"])
            self.assertFalse(report["motion_attempted"])
            self.assertEqual(report["trials"], [])
            self.assertEqual(report["final"]["fault"], 0x5210)
        self.assertFalse(bench.active)
