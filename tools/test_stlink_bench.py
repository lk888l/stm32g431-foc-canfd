#!/usr/bin/env python3
"""Transport-free checks for the attended bench command sequences and cleanup."""
import contextlib
import io
import json
from pathlib import Path
import struct
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import stlink_bench


class SimulatedBench(stlink_bench.Bench):
    def __init__(self, *_):
        self.events = []
        self.status = 0x40
        self.position = 3.9
        self.velocity = 0
        self.active = False

    def __enter__(self):
        return self

    def __exit__(self, *_):
        pass

    def stop(self):
        self.events.append(("stop",))
        self.active = False
        self.status = 0x40

    def snapshot(self):
        self.events.append(("snapshot", self.active))
        return dict(status=self.status, position=self.position, velocity=self.velocity,
                    iq=0.0, id=0.0, current_limit=0.25, calibration=4, fault=0)

    def nmt(self, command):
        self.events.append(("nmt", command))

    def sdo(self, index, sub=0, value=None, size=4, fmt=None):
        self.events.append(("sdo", index, sub, value, size, fmt))
        if value is None and index == 0x6041:
            return struct.pack("<I", self.status)
        if index == 0x6040:
            self.status = {6: 0x21, 7: 0x23, 15: 0x27, 31: 0x1027, 95: 0x1027}[value]
            self.active = self.status & 0x6F == 0x27
        return b"\0" * 4


class BenchCommandsTest(unittest.TestCase):
    def test_single_turn_shortest_path_and_half_turn_tie(self):
        self.assertAlmostEqual(stlink_bench.position_distance(3.9, 0.1, True), 0.2)
        self.assertAlmostEqual(stlink_bench.position_distance(3.1, 0.9, True), -0.2)
        self.assertEqual(stlink_bench.position_distance(3, 0.5, True), -0.5)
        self.assertAlmostEqual(stlink_bench.position_distance(3.9, 1.2, True, True), 0.2)
        self.assertAlmostEqual(stlink_bench.position_distance(3.9, 1.2, False, True), 1.2)

    def test_far_multi_turn_target_rejected_before_enable(self):
        bench = SimulatedBench()
        with self.assertRaisesRegex(ValueError, "travel"):
            bench.enable_position(20)
        self.assertFalse(bench.active)
        self.assertNotIn(("nmt", 1), bench.events)

    def test_enable_uses_lightweight_status_reads(self):
        bench = SimulatedBench()
        bench.enable_velocity()
        self.assertTrue(bench.active)
        self.assertNotIn(("snapshot", True), bench.events)
        reads = [event for event in bench.events if event[:3] == ("sdo", 0x6041, 0)]
        self.assertEqual(len(reads), 3)

    def test_relative_position_refresh_does_not_retrigger(self):
        bench = SimulatedBench()
        with patch.object(stlink_bench, "Bench", return_value=bench), contextlib.redirect_stdout(io.StringIO()):
            stlink_bench.main(["multi-position", "1.25", "--relative", "--seconds", "1"])
        targets = [event for event in bench.events if event[:2] == ("sdo", 0x607A)]
        self.assertEqual(len(targets), 1)
        controlwords = [event[3] for event in bench.events if event[:2] == ("sdo", 0x6040)]
        self.assertEqual(controlwords[:3], [6, 7, 15])
        self.assertTrue(controlwords[3:] and all(value == 0x5F for value in controlwords[3:]))
        self.assertFalse(bench.active)

    def test_mit_is_complete_before_enable_and_disables_packed_mode(self):
        bench = SimulatedBench()
        initial, target = bench.enable_mit(position=0.1, relative=True, kp=0.0104, kd=0.0024)
        self.assertAlmostEqual(target, initial["position"] + 0.1)
        setup = bench.events[:bench.events.index(("nmt", 1))]
        self.assertIn(("sdo", 0x2004, 1, 0, 1, None), setup)
        for sub, value, size, fmt in ((1, target, 4, "<f"), (2, 0, 4, "<f"),
                                     (3, 0, 4, "<f"), (4, 10, 2, None), (5, 2, 2, None)):
            self.assertIn(("sdo", 0x2003, sub, value, size, fmt), setup)

    def test_records_preserve_pre_motion_origin_and_resolve_position_target(self):
        for mode, target, relative, expected in (
            ("position", 0.1, False, 4.1),
            ("multi-position", 1.25, True, 5.15),
            ("multi-position", 1.25, False, 1.25),
        ):
            with self.subTest(mode=mode, relative=relative):
                bench = SimulatedBench()
                args = SimpleNamespace(operation=mode, target=target, relative=relative, speed=1, seconds=1)
                records = []
                # Activation and ACK polling also consume monotonic reads; advance only in sleep.
                clock = [0.0]
                original_snapshot = bench.snapshot
                def moving_snapshot():
                    state = original_snapshot()
                    if bench.active:
                        state["position"] += 0.03
                    return state
                with patch.object(stlink_bench.time, "monotonic", side_effect=lambda: clock[0]), \
                     patch.object(bench, "snapshot", side_effect=moving_snapshot), \
                     patch.object(stlink_bench.time, "sleep", side_effect=lambda _: clock.__setitem__(0, 2)):
                    stlink_bench.run_motion(bench, args, records)
                self.assertEqual(len(records), 1)
                self.assertEqual(records[0]["initial_position"], 3.9)
                self.assertAlmostEqual(records[0]["position"], 3.93)
                self.assertAlmostEqual(records[0]["resolved_target"], expected)

    def test_invalid_inputs_never_open_transport(self):
        cases = (["velocity", "nan"], ["velocity", "1", "--seconds", "inf"],
                 ["torque", "201"], ["position", "0", "--speed", "0"],
                 ["mit", "--kp", "nan"], ["mit", "--torque", "0.006"])
        for argv in cases:
            with self.subTest(argv=argv), patch.object(stlink_bench, "Bench") as bench:
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    stlink_bench.main(argv)
                bench.assert_not_called()

    def test_overspeed_stops_and_preserves_fault_sample(self):
        bench = SimulatedBench()
        bench.velocity = 8
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "trial.json"
            with patch.object(stlink_bench, "Bench", return_value=bench):
                with self.assertRaisesRegex(RuntimeError, "Motion trial aborted"):
                    stlink_bench.main(["--output", str(output), "torque", "100"])
            self.assertFalse(bench.active)
            records = json.loads(output.read_text(encoding="utf8"))
            self.assertEqual(records[-1]["velocity"], 8)
            self.assertEqual(records[-1]["target_iq"], 0.025)

    def test_stop_transport_failure_still_saves_telemetry(self):
        bench = SimulatedBench()
        def fail_trial(_bench, _args, records):
            records.append({"fault": 0x8130})
            raise RuntimeError("trial failure")
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "trial.json"
            with patch.object(stlink_bench, "Bench", return_value=bench), \
                 patch.object(stlink_bench, "run_motion", side_effect=fail_trial), \
                 patch.object(bench, "stop", side_effect=ConnectionError("stop failure")):
                with self.assertRaises(ConnectionError):
                    stlink_bench.main(["--output", str(output), "torque", "100"])
            self.assertEqual(json.loads(output.read_text(encoding="utf8")), [{"fault": 0x8130}])

    def test_snapshot_waits_for_new_sequence_before_bulk_read(self):
        bench = stlink_bench.Bench.__new__(stlink_bench.Bench)
        bench.state, bench.request = 0x20000000, 0x20000100
        polls = iter((2, 2, 3, 4, 4))
        calls = []
        words = [0x524C4142, 2, 4] + [0] * 28
        def read(address, count):
            calls.append((address, count))
            if address == bench.state + 8:
                return [next(polls)]
            if address == bench.request:
                return [10]
            return words
        with patch.object(bench, "read", side_effect=read), patch.object(bench, "write"):
            bench.snapshot()
        bulk = calls.index((bench.state, 31))
        self.assertEqual(sum(call == (bench.state + 8, 1) for call in calls[:bulk]), 4)
        self.assertEqual(calls.count((bench.state, 31)), 1)


if __name__ == "__main__":
    unittest.main()
