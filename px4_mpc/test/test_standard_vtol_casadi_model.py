"""Equivalence tests for the numerical and CasADi transition models."""

import unittest

import casadi as cs
import numpy as np

from px4_mpc.models.standard_vtol_casadi_model import (
    StandardVtolTransitionCasadiModel,
)
from px4_mpc.models.standard_vtol_gz_model import StandardVtolTransitionRateModel


class TestStandardVtolTransitionCasadiModel(unittest.TestCase):
    def setUp(self):
        self.numeric = StandardVtolTransitionRateModel()
        self.symbolic = StandardVtolTransitionCasadiModel(self.numeric.plant)
        self.function = self.symbolic.function()

    def compare(self, state, control, wind, elevator=0.0):
        expected = self.numeric.derivative(state, control, wind, elevator)
        parameters = np.r_[wind, elevator]
        actual = np.asarray(self.function(state, control, parameters)).reshape(-1)
        np.testing.assert_allclose(actual, expected, rtol=1.0e-9, atol=1.0e-9)

    def test_hover_equilibrium(self):
        self.compare(
            self.numeric.hover_state(), self.numeric.hover_control(), np.zeros(3)
        )

    def test_transition_samples_match(self):
        samples = (
            (8.0, -8.0, 0.28, 0.29, [0.05, -0.1, 0.02], [0.0, 0.0, 0.0], 0.4),
            (15.0, -1.4, 0.0, 0.267, [-0.03, 0.08, -0.01], [1.5, -0.5, 0.0], 0.2),
            (22.0, 1.2, 0.0, 0.273, [0.0, 0.0, 0.0], [-2.0, 0.7, 0.0], 0.04),
        )
        for speed, pitch_deg, lift, pusher, rates, wind, elevator in samples:
            pitch = np.deg2rad(pitch_deg)
            state = np.zeros(10)
            state[3] = speed
            state[6:10] = [np.cos(pitch / 2), 0.0, np.sin(pitch / 2), 0.0]
            control = np.array([lift, pusher, *rates])
            with self.subTest(speed=speed):
                self.compare(state, control, np.asarray(wind), elevator)

    def test_level_forward_flight_jacobian_is_finite(self):
        state, control, parameters, dynamics = self.symbolic.symbolic_dynamics()
        jacobian = cs.Function(
            "level_flight_jacobian",
            [state, control, parameters],
            [cs.jacobian(dynamics, state)],
        )
        level_state = self.numeric.hover_state()
        level_state[3] = 2.0
        actual = np.asarray(
            jacobian(
                level_state,
                self.numeric.hover_control(),
                np.zeros(4),
            )
        )
        self.assertTrue(np.all(np.isfinite(actual)))


if __name__ == "__main__":
    unittest.main()
