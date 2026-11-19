"""Measurement-window and abort checks without attached hardware."""
import math
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import motor_trace
from motor_trace import check_state, summarize, validate
from test_stlink_bench import SimulatedBench


class TraceTests(unittest.TestCase):
    def args(self, **changes):
        values = dict(target=10, seconds=12, current_limit=0.3, acceleration=2, settle=6)
        return SimpleNamespace(**(values | changes))

    def test_rejects_unreachable_window_and_nonfinite_settings(self):
        validate(self.args())
        for changes in (dict(seconds=5), dict(target=41), dict(current_limit=math.nan),
                        dict(acceleration=0), dict(settle=math.inf)):
            with self.assertRaises(ValueError):
                validate(self.args(**changes))

    def test_checks_vector_current_and_motion_even_when_status_is_good(self):
        state = dict(fault=0, status=0x27, bus=19.3, temperature=35,
                     position=1, velocity=10, iq=0.1, id=0)
        check_state(state, [], 10, 0.3)
        for change in (dict(iq=0.4, id=0.4), dict(velocity=13), dict(position=math.nan),
                       dict(bus=36), dict(temperature=math.nan), dict(status=0x40), dict(fault=1)):
            with self.assertRaises(RuntimeError):
                check_state(state | change, [], 10, 0.3)
        with self.assertRaises(RuntimeError):
            check_state(state, [state | dict(iq=2)], 10, 0.3)

    def test_position_derivative_never_crosses_capture_gaps_or_counter_wrap(self):
        captures = []
        origin = 0xFFFF0000
        for start in (0, 6000, 6200):
            captures.append([dict(samples=(origin + (start+i)*20) & 0xFFFFFFFF,
                                  position=100 + 2*(start+i)/1000, velocity=2 if start else 5,
                                  iq=0.1, id=0, iq_target=0.1) for i in range(96)])
        report = dict(captures=captures, target=2, settle=5, current_limit=0.3)
        result = summarize(report)
        self.assertEqual(result["steady_samples"], 192)
        self.assertEqual(result["velocity_mean"], 2)
        self.assertEqual(result["velocity_std"], 0)
        self.assertLess(result["position_velocity_10ms_std"], 1e-9)
        self.assertEqual(result["iq_error_rms"], 0)
        self.assertEqual(result["current_limit_fraction"], 0)

    def test_does_not_claim_metrics_with_no_steady_window(self):
        with self.assertRaises(ValueError):
            summarize(dict(captures=[]))

    def test_failure_stops_drive_and_preserves_a_failed_report(self):
        bench = SimulatedBench()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "trace.json"
            with patch.object(motor_trace, "Bench", return_value=bench), \
                 patch.object(motor_trace, "run_trace", side_effect=RuntimeError("test abort")):
                with self.assertRaisesRegex(RuntimeError, "test abort"):
                    motor_trace.main(["1", "--output", str(output)])
            report = json.loads(output.read_text())
            self.assertFalse(report["completed"])
            self.assertEqual(report["error"], "test abort")
            self.assertIn("final", report)
        self.assertIn(("stop",), bench.events)


if __name__ == "__main__":
    unittest.main()
