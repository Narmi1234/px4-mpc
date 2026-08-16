"""Tests for live/offline Standard VTOL output safety functions."""

import unittest

import numpy as np

from px4_mpc.controllers.standard_vtol_output import (
    limit_mc_command,
    vertical_hover_lift,
)
from px4_mpc.models.standard_vtol_gz_model import StandardVtolGazeboModel


class TestStandardVtolOutput(unittest.TestCase):
    def test_limiter_forces_pusher_zero_and_applies_slew(self):
        previous = np.array([0.52, 0.0, 0.0, 0.0, 0.0])
        requested = np.array([0.65, 1.0, 1.0, -1.0, 1.0])
        actual = limit_mc_command(previous, requested, dt=0.05)
        np.testing.assert_allclose(
            actual,
            [0.525, 0.0, 0.015, -0.015, 0.01],
            atol=1.0e-12,
        )

    def test_vertical_law_is_neutral_at_hover(self):
        plant = StandardVtolGazeboModel()
        self.assertAlmostEqual(
            vertical_hover_lift(plant, 0.0, 0.0), plant.hover_command
        )
        self.assertLess(vertical_hover_lift(plant, 0.1, 0.0), plant.hover_command)
        self.assertGreater(vertical_hover_lift(plant, -0.1, 0.0), plant.hover_command)


if __name__ == "__main__":
    unittest.main()
