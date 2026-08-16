"""Tests for the Standard VTOL transition trim corridor."""

import unittest

import numpy as np

from px4_mpc.models.standard_vtol_trim import StandardVtolTrimSolver


class TestStandardVtolTrimSolver(unittest.TestCase):
    def test_corridor_is_force_balanced_and_bounded(self):
        solver = StandardVtolTrimSolver()
        corridor = solver.corridor(np.array([0.0, 5.0, 10.0, 15.0, 22.0]), 0.5)
        self.assertEqual(len(corridor), 5)
        for point in corridor:
            self.assertLess(abs(point.acceleration_world[0]), 1.0e-5)
            self.assertLess(abs(point.acceleration_world[2]), 1.0e-5)
            self.assertGreaterEqual(point.collective_lift, solver.lift_min)
            self.assertLessEqual(point.collective_lift, solver.lift_max)
            self.assertGreaterEqual(point.pusher, solver.pusher_min)
            self.assertLessEqual(point.pusher, solver.pusher_max)
            self.assertGreaterEqual(point.elevator, -0.78)
            self.assertLessEqual(point.elevator, 0.78)
            if point.airspeed >= 5.0:
                self.assertLess(abs(point.required_pitch_acceleration), 1.0e-5)

    def test_corridor_starts_at_hover_and_unloads_lift_rotors(self):
        solver = StandardVtolTrimSolver()
        corridor = solver.corridor(np.array([0.0, 10.0, 15.0, 22.0]), 0.5)
        self.assertAlmostEqual(corridor[0].pitch, 0.0)
        self.assertAlmostEqual(corridor[0].collective_lift, solver.plant.hover_command)
        self.assertAlmostEqual(corridor[0].pusher, 0.0)
        self.assertLess(corridor[-1].collective_lift, corridor[0].collective_lift)
        self.assertGreater(corridor[-1].pusher, corridor[0].pusher)
        lift = np.array([point.collective_lift for point in corridor])
        self.assertTrue(np.all(np.diff(lift) <= 1.0e-6))


if __name__ == "__main__":
    unittest.main()
