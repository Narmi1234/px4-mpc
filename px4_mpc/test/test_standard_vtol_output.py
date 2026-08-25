"""Tests for live/offline Standard VTOL output safety functions."""

import unittest

import numpy as np

from px4_mpc.controllers.standard_vtol_output import (
    govern_pusher_forward_envelope,
    limit_external_pusher_command,
    limit_mc_command,
    limit_pusher_forward_command,
    vertical_hover_lift,
)
from px4_mpc.models.standard_vtol_gz_model import StandardVtolGazeboModel


class TestStandardVtolOutput(unittest.TestCase):
    def test_external_pusher_limiter_allows_only_slow_bounded_pulse(self):
        previous = np.array([0.52, 0.0, 0.0, 0.0, 0.0])
        requested = np.array([0.52, 0.4, 0.0, 0.0, 0.0])
        actual = limit_external_pusher_command(
            previous, requested, pusher_command=0.2, dt=0.05
        )
        self.assertAlmostEqual(actual[1], 0.001)
        current = actual
        for _ in range(100):
            current = limit_external_pusher_command(
                current, requested, pusher_command=0.2, dt=0.05
            )
        self.assertAlmostEqual(current[1], 0.05)

    def test_limiter_forces_pusher_zero_and_applies_slew(self):
        previous = np.array([0.52, 0.0, 0.0, 0.0, 0.0])
        requested = np.array([0.65, 1.0, 1.0, -1.0, 1.0])
        actual = limit_mc_command(previous, requested, dt=0.05)
        np.testing.assert_allclose(
            actual,
            [0.525, 0.0, 0.015, -0.015, 0.01],
            atol=1.0e-12,
        )

    def test_pusher_forward_limiter_uses_separate_bounded_envelope(self):
        previous = np.array([0.52, 0.0, 0.0, 0.0, 0.0])
        requested = np.array([0.65, 0.8, 1.0, -1.0, 1.0])
        actual = limit_pusher_forward_command(previous, requested, dt=0.05)
        np.testing.assert_allclose(
            actual,
            [0.525, 0.0015, 0.01, -0.015, 0.01],
            atol=1.0e-12,
        )
        current = actual
        for _ in range(100):
            current = limit_pusher_forward_command(current, requested, dt=0.05)
        self.assertAlmostEqual(current[1], 0.10)
        self.assertLessEqual(current[0], 0.56)
        self.assertLessEqual(abs(current[2]), 0.10)
        self.assertLessEqual(abs(current[3]), 0.20)
        self.assertLessEqual(abs(current[4]), 0.15)

    def test_vertical_law_is_neutral_at_hover(self):
        plant = StandardVtolGazeboModel()
        self.assertAlmostEqual(
            vertical_hover_lift(plant, 0.0, 0.0), plant.hover_command
        )
        self.assertLess(vertical_hover_lift(plant, 0.1, 0.0), plant.hover_command)
        self.assertGreater(vertical_hover_lift(plant, -0.1, 0.0), plant.hover_command)

    def test_vertical_law_is_critically_damped_at_two_radians_squared(self):
        plant = StandardVtolGazeboModel()
        position_only = vertical_hover_lift(plant, 0.20, 0.0)
        rising = vertical_hover_lift(plant, 0.20, 0.10)
        falling = vertical_hover_lift(plant, 0.20, -0.10)
        self.assertLess(rising, position_only)
        self.assertGreater(falling, rising)

    def test_pusher_governor_is_inactive_inside_tracking_band(self):
        previous = np.array([0.52, 0.06, 0.0, 0.02, 0.0])
        limited = np.array([0.52, 0.0615, 0.0, 0.03, 0.0])
        actual = govern_pusher_forward_envelope(
            previous, limited, 2.0, 1.95, 3.0, 0.05, 0.05
        )
        np.testing.assert_allclose(actual, limited)

    def test_pusher_governor_removes_thrust_and_requests_leveling(self):
        previous = np.array([0.52, 0.06, 0.0, 0.02, 0.0])
        limited = np.array([0.52, 0.0615, 0.0, 0.03, 0.0])
        actual = govern_pusher_forward_envelope(
            previous, limited, 3.2, 2.9, 3.0, np.deg2rad(5.0), 0.05
        )
        self.assertAlmostEqual(actual[1], 0.055)
        self.assertAlmostEqual(actual[3], -8.0 * np.deg2rad(1.0))
        self.assertLess(actual[3], limited[3])

    def test_pusher_governor_limits_forward_pitch_before_hard_tilt(self):
        previous = np.array([0.52, 0.08, 0.0, 0.12, 0.0])
        limited = np.array([0.52, 0.0815, 0.0, 0.135, 0.0])
        actual = govern_pusher_forward_envelope(
            previous, limited, 2.5, 2.7, 3.0, np.deg2rad(6.0), 0.05
        )
        self.assertAlmostEqual(actual[1], 0.0815)
        self.assertAlmostEqual(actual[3], -0.2)
        self.assertLess(actual[3], limited[3])



if __name__ == "__main__":
    unittest.main()
