import math
import unittest

try:
    import numpy as np
    from fit_cogging import fit
except ImportError:
    np = None


@unittest.skipIf(np is None, "Optional numpy is needed only for offline map fitting")
class CoggingFitTest(unittest.TestCase):
    @staticmethod
    def scan():
        rows = []
        for forward in (True, False):
            for n in range(1800):
                p = (n + 0.5) / 1800
                if not forward:
                    p = 1 - p
                # Rotate known rotor-frame current into the commanded frame.
                delta = 0.1 * math.sin(2 * math.pi * p)
                q = 0.004 + (0.03 if forward else -0.03) + 0.06 * math.sin(44 * math.pi * p)
                d = 0.4
                rows.append(dict(calibration=2, warnings=0,
                                 calibration_ticks=int((n + 0.5) * 2000000 / 1800) + (0 if forward else 2000000),
                                 position=p, calibration_angle=p * 22 * math.pi - 3.1 + delta,
                                 id=d * math.cos(delta) + q * math.sin(delta),
                                 iq=-d * math.sin(delta) + q * math.cos(delta)))
        rows.append(dict(calibration=4, fault=0, pole_pairs=11, direction=1, offset=3.1))
        return rows

    def test_rotation_and_bidirectional_friction_separation(self):
        table, summary = fit(self.scan())
        expected = 0.06 * np.sin(2 * np.pi * 22 * np.arange(1024) / 1024)
        self.assertLess(np.max(np.abs(table - expected)), 0.001)
        self.assertAlmostEqual(summary["bias_a"], 0.004, places=3)
        self.assertAlmostEqual(summary["friction_a"], 0.03, places=3)
        self.assertLess(abs(np.mean(table)), 1e-6)

    def test_failed_scan_and_missing_angular_coverage_rejected(self):
        rows = self.scan()
        rows[-1]["fault"] = 1
        with self.assertRaisesRegex(ValueError, "successfully completed"):
            fit(rows)
        rows[-1]["fault"] = 0
        rows = [r for r in rows if r.get("position", 0) < 0.8]
        with self.assertRaisesRegex(ValueError, "complete revolution"):
            fit(rows)


if __name__ == "__main__":
    unittest.main()
