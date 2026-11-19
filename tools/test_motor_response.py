import math
import struct
import unittest
from unittest.mock import Mock

from motor_response import settling_ms, spectrum, step_metrics
from stlink_bench import Bench


class ResponseTests(unittest.TestCase):
    def test_wrapped_negative_origin_and_command_markers(self):
        b = Bench.__new__(Bench)
        b.symbols = {"roller_trace_first": 1, "roller_trace_last": 2, "roller_trace_origin": 3,
                     "roller_trace_request": 4}
        start = 0xfffff000
        origin = -10000 * 16384
        memory = {1: [start], 2: [(start + 767*20) & 0xffffffff],
                  3: struct.unpack("<II", struct.pack("<q", origin)), 4: [2]}
        b.read = b.frozen_words = lambda addr, _: memory[addr]
        b.arm_capture = Mock(return_value=2)
        b.frozen_capture = Mock(return_value=b"".join(struct.pack("<Hh", ((origin-i) & 16383) |
                                  (0xc000 if i >= 80 else 0), -512) for i in range(768)))
        rows = b.motion_capture()
        self.assertEqual(rows[-1]["counts"], origin-767)
        self.assertTrue(rows[80]["ack"] and rows[80]["marked"])
        self.assertFalse(rows[79]["ack"] or rows[79]["marked"])
        self.assertEqual(rows[-1]["velocity"], -1)

    def test_band_power_for_unattenuated_90hz_velocity(self):
        try:
            import numpy  # noqa: F401
        except ImportError:
            self.skipTest("numpy unavailable")
        rows = [dict(velocity=6 + math.sin(math.tau * 90*i/1000)/60, counts=100*i) for i in range(768)]
        result = spectrum([rows, rows])
        self.assertAlmostEqual(result["band_80_100_rms_rpm"], math.sqrt(.5), places=4)
        self.assertLess(abs(result["band_peak_hz"] - 90), 1)
        self.assertEqual(result["encoder_10ms_band_rms_rpm"], 0.)

    def test_current_peak_stays_with_its_mask_and_pwm_frame_across_tick_wrap(self):
        b = Bench.__new__(Bench)
        b.symbols = {"roller_trace_first": 1, "roller_trace_last": 2, "roller_trace_request": 3}
        first = 0xfffffff0
        memory = {1: [first], 2: [(first+1999) & 0xffffffff], 3: [9]}
        b.read = b.frozen_words = lambda address, _: memory[address]
        b.arm_capture = Mock(return_value=9)
        data = bytearray(8*56)
        struct.pack_into("<II10f4H", data, 5*56, 2000, 17, 500., .25, .3, -.4, .1,
                         -.2, .15, .1, 2., 19.3, 1500, 3950, 50, 0)
        b.frozen_capture = Mock(return_value=data)
        capture = b.peak_capture()
        peak, = capture["peaks"]
        self.assertEqual((peak["mask"], peak["frames"], peak["ccr"]), (5, 2000, [1500,3950,50]))
        self.assertAlmostEqual(peak["peak"], .5)
        struct.pack_into("<I", data, 5*56+4, 2500)
        with self.assertRaisesRegex(RuntimeError, "same-frame"):
            b.peak_capture()

    def test_settling_uses_actual_edge_and_excludes_last_moment_entries(self):
        self.assertIsNone(settling_ms([1, 0, 0], .1))
        self.assertEqual(settling_ms([1, 0, 1] + [0]*50, .1), 3)
        rows = [dict(counts=8000+(17 if i >= 110 else 0), ack=i >= 80) for i in range(300)]
        self.assertEqual(step_metrics(rows, "position", 17/16384)["within_0p1deg_ms"], 30)
