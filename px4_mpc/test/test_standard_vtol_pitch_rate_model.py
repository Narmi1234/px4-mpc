import unittest

import numpy as np

from px4_mpc.models.standard_vtol_pitch_rate_model import (
    StandardVtolPitchRateLpvModel,
)


class TestStandardVtolPitchRateLpvModel(unittest.TestCase):
    def test_basis_is_convex_at_all_scheduling_points(self):
        for airspeed in (-5.0, 0.0, 10.0, 20.0, 30.0):
            for lift in (-1.0, 0.0, 0.5, 1.0, 2.0):
                basis = StandardVtolPitchRateLpvModel.scheduling_basis(
                    airspeed, lift
                )
                self.assertTrue(np.all(basis >= 0.0))
                self.assertAlmostEqual(float(np.sum(basis)), 1.0)

    def test_scheduled_damping_is_strictly_positive(self):
        for airspeed in np.linspace(0.0, 25.0, 11):
            for lift in np.linspace(0.0, 1.0, 11):
                damping, input_gain = StandardVtolPitchRateLpvModel.coefficients(
                    airspeed, lift
                )
                self.assertGreaterEqual(damping, 0.02 - 1.0e-12)
                self.assertGreaterEqual(input_gain, 0.0)

    def test_disturbance_is_bounded(self):
        model = StandardVtolPitchRateLpvModel
        positive = model.derivative(0.0, 0.0, 15.0, 1.0, 0.0, 0.0, 10.0)
        negative = model.derivative(0.0, 0.0, 15.0, 1.0, 0.0, 0.0, -10.0)
        self.assertAlmostEqual(positive, model.pitch_disturbance_bound)
        self.assertAlmostEqual(negative, -model.pitch_disturbance_bound)

    def test_mc_rate_response_has_correct_sign(self):
        derivative = StandardVtolPitchRateLpvModel.derivative(
            pitch_rate=0.0,
            pitch_rate_setpoint=0.1,
            airspeed=2.0,
            lift_fraction=1.0,
            angle_of_attack=0.0,
            pitch=0.0,
        )
        self.assertGreater(derivative, 0.0)


if __name__ == "__main__":
    unittest.main()
