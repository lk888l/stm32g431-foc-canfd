import math
import unittest

from fit_back_emf import fit


class BackEmfFitTests(unittest.TestCase):
    def test_separates_voltage_drop_from_back_emf_in_both_directions(self):
        groups = [dict(omega=w, emf=0.104*w+math.copysign(0.12, w)) for w in (40, 75, -40, -75)]
        result = fit(groups)
        self.assertAlmostEqual(result["ke_v_per_mechanical_rad_s"], 0.104)
        self.assertAlmostEqual(result["odd_voltage_offset"], 0.12)
        self.assertLess(result["residual_rms_v"], 1e-12)

    def test_rejects_unidentifiable_or_single_direction_data(self):
        for speeds in ((40, 40, -40, -40), (40, 60, 75, 80), (40, -75)):
            with self.assertRaises(ValueError):
                fit([dict(omega=w, emf=w*.104) for w in speeds])


if __name__ == "__main__":
    unittest.main()
