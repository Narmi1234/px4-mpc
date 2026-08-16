"""Regression tests for the Gazebo-derived standard VTOL model."""

import unittest

import numpy as np

from px4_mpc.models.standard_vtol_gz_model import (
    StandardVtolGazeboModel,
    StandardVtolTransitionRateModel,
)


class TestStandardVtolGazeboModel(unittest.TestCase):
    def setUp(self):
        self.model = StandardVtolGazeboModel()

    def test_parameters_match_px4_sdf_snapshot(self):
        self.assertAlmostEqual(self.model.mass, 5.02500003, places=8)
        np.testing.assert_allclose(
            np.diag(self.model.base_inertia_b),
            [0.477708333333, 0.341666666667, 0.811041666667],
            rtol=0.0,
            atol=1.0e-12,
        )
        self.assertAlmostEqual(self.model.motors[0].motor_constant, 2.0e-5)
        self.assertAlmostEqual(self.model.motors[4].motor_constant, 8.54858e-6)

    def test_allocated_hover_is_a_full_rigid_body_equilibrium(self):
        derivative = self.model.derivative(
            self.model.hover_state(),
            self.model.hover_control(),
        )
        np.testing.assert_allclose(derivative, np.zeros(18), atol=1.0e-10)
        self.assertAlmostEqual(self.model.hover_command, 0.520119535, places=8)

    def test_previous_high_command_accelerates_upward(self):
        state = self.model.hover_state()
        state[13:17] = [self.model.motors[index].target_speed(0.525) for index in range(4)]
        control = self.model.hover_control()
        control[0:4] = 0.525
        derivative = self.model.derivative(state, control)
        self.assertGreater(derivative[5], 0.18)

    def test_wing_produces_lift_and_drag_in_forward_flight(self):
        force_b, _ = self.model.aerodynamic_wrench(
            np.zeros(3),
            np.array([15.0, 0.0, 0.0]),
            np.zeros(3),
        )
        self.assertLess(force_b[0], 0.0)
        self.assertGreater(force_b[2], 35.0)

    def test_level_trim_balances_drag_and_weight_below_wingborne_speed(self):
        trim = self.model.nominal_level_flight_trim(15.0)
        self.assertGreater(trim.collective_lift_command, 0.0)
        self.assertGreater(trim.pusher_command, 0.0)
        np.testing.assert_allclose(trim.total_force_b[0], 0.0, atol=1.0e-12)
        np.testing.assert_allclose(
            trim.total_force_b[2],
            self.model.mass * self.model.gravity,
            atol=1.0e-12,
        )

    def test_surface_commands_obey_sdf_joint_limits(self):
        velocity = np.array([15.0, 0.0, 0.0])
        force_limited, torque_limited = self.model.aerodynamic_wrench(
            np.array([0.78, -0.78, 0.78]), velocity, np.zeros(3)
        )
        force_excess, torque_excess = self.model.aerodynamic_wrench(
            np.array([10.0, -10.0, 10.0]), velocity, np.zeros(3)
        )
        np.testing.assert_allclose(force_excess, force_limited)
        np.testing.assert_allclose(torque_excess, torque_limited)

    def test_reduced_rate_model_has_hover_equilibrium(self):
        reduced = StandardVtolTransitionRateModel(self.model)
        derivative = reduced.derivative(reduced.hover_state(), reduced.hover_control())
        np.testing.assert_allclose(derivative, np.zeros(10), atol=1.0e-10)

    def test_reduced_rate_model_accepts_scheduled_elevator_trim(self):
        reduced = StandardVtolTransitionRateModel(self.model)
        state = reduced.hover_state()
        state[3] = 15.0
        neutral = reduced.derivative(state, reduced.hover_control())
        trimmed = reduced.derivative(
            state, reduced.hover_control(), elevator_trim=np.deg2rad(12.0)
        )
        self.assertLess(trimmed[5], neutral[5])


if __name__ == "__main__":
    unittest.main()
