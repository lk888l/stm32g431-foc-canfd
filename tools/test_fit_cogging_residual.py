import unittest

try:
    import numpy as np
    from fit_cogging_residual import angle_fit, basis
except ImportError:
    np = None


@unittest.skipIf(np is None, "numpy unavailable")
class ResidualFitTests(unittest.TestCase):
    def test_direction_bias_does_not_enter_periodic_map(self):
        # Oversample one quadrant to exercise the equal-angle weighting.
        phase = np.r_[np.arange(1024)/1024, np.tile(np.arange(256)/1024, 3)]
        periodic = .035*np.cos(4*np.pi*phase) + .008*np.sin(44*np.pi*phase)
        data = [(phase, periodic+.003+sign*.042, sign*.1, "") for sign in (1,-1)]
        forward, reverse = (angle_fit(data, sign) for sign in (1,-1))
        mean = (forward+reverse)/2
        self.assertAlmostEqual((forward[0]-reverse[0])/2, .042, places=5)
        self.assertAlmostEqual(mean[0], .003, places=5)
        mean[0] = 0
        self.assertLess(np.max(abs(basis(phase) @ mean - periodic)), .0002)

    def test_requires_both_directions_and_angle_coverage(self):
        phase = np.arange(1100)/4096
        block = (phase, np.zeros(len(phase)), .1, "")
        for sign in (-1, 1):
            with self.assertRaises(ValueError):
                angle_fit([block], sign)
