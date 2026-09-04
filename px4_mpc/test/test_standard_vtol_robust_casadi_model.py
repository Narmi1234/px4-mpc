import unittest

import casadi as cs
import numpy as np

from px4_mpc.models.standard_vtol_robust_casadi_model import (
    StandardVtolRobustCasadiModel,
)


class TestStandardVtolRobustCasadiModel(unittest.TestCase):
    def setUp(self):
        self.model = StandardVtolRobustCasadiModel()
        self.function = self.model.function()

    def test_dimensions_and_finite_hover(self):
        value = np.asarray(self.function(
            self.model.hover_state(), self.model.hover_control(),
            self.model.nominal_parameters()
        )).reshape(-1)
        self.assertEqual(value.shape, (16,))
        self.assertTrue(np.all(np.isfinite(value)))
        self.assertLess(np.linalg.norm(value[3:6]), 0.1)

    def test_pitch_disturbance_has_frd_to_flu_sign(self):
        positive_frd = self.model.nominal_parameters()
        positive_frd[5] = 0.2
        derivative = np.asarray(self.function(
            self.model.hover_state(), self.model.hover_control(), positive_frd
        )).reshape(-1)
        self.assertAlmostEqual(derivative[11], -0.2, places=9)

    def test_lambda_zero_removes_vertical_motor_lift(self):
        state = self.model.hover_state()
        control = self.model.hover_control()
        parameters = self.model.nominal_parameters()
        full = np.asarray(self.function(state, control, parameters)).reshape(-1)
        control[5] = 0.0
        no_lift = np.asarray(self.function(state, control, parameters)).reshape(-1)
        self.assertLess(no_lift[5], full[5] - 5.0)

    def test_symbolic_jacobians_are_finite(self):
        state, control, parameters, dynamics = self.model.symbolic_dynamics()
        jacobian = cs.Function(
            "robust_vtol_jacobian",
            [state, control, parameters],
            [cs.jacobian(dynamics, state), cs.jacobian(dynamics, control)],
        )
        x = self.model.hover_state()
        x[3] = 8.0
        values = jacobian(
            x, self.model.hover_control(), self.model.nominal_parameters()
        )
        self.assertTrue(all(np.all(np.isfinite(np.asarray(value))) for value in values))

    def test_fixed_wing_elevator_trim_is_scheduled(self):
        speed = cs.MX.sym("speed")
        trim = cs.Function("trim", [speed], [self.model._elevator_trim(speed)])
        self.assertAlmostEqual(
            float(trim(10.0)), np.deg2rad(41.6336729723), places=8
        )
        self.assertAlmostEqual(
            float(trim(15.0)), np.deg2rad(12.9051872155), places=8
        )

    def test_l4_roll_prediction_uses_identified_aileron_moment(self):
        state = self.model.hover_state()
        state[3] = 12.0
        state[10] = 0.20
        state[13] = -0.20
        state[14] = 0.20
        control = self.model.hover_control()
        parameters = self.model.nominal_parameters()
        parameters[6] = 0.05
        derivative = np.asarray(
            self.function(state, control, parameters)
        ).reshape(-1)
        expected_aero = (
            -0.95 * self.model.roll_aero_damping * state[10]
            + self.model.roll_surface_gain * 12.0 ** 2
            * ((state[14] - state[13]) / (2.0 * np.pi / 4.0))
        )
        expected_mc = 0.05 * self.model.roll_rate_gain * (-state[10])
        self.assertAlmostEqual(derivative[10], expected_mc + expected_aero)

    def test_l4_yaw_target_is_coordinated_with_bank(self):
        state = self.model.hover_state()
        state[3] = 12.0
        roll = np.deg2rad(8.0)
        state[6] = np.cos(0.5 * roll)
        state[7] = np.sin(0.5 * roll)
        parameters = self.model.nominal_parameters()
        parameters[6] = 0.05
        derivative = np.asarray(
            self.function(state, self.model.hover_control(), parameters)
        ).reshape(-1)
        coordinated = (
            -self.model.coordinated_turn_gain * self.model.plant.gravity
            * np.tan(roll) / 12.0
        )
        self.assertAlmostEqual(
            derivative[12],
            self.model.yaw_rate_gain * 0.95 * coordinated,
            places=7,
        )

    def test_l4_pitch_dynamics_use_torque_not_lift_weight(self):
        state = self.model.hover_state()
        state[3] = 12.0
        state[11] = 0.15
        control = self.model.hover_control()
        control[5] = 0.20
        full_mc = self.model.nominal_parameters()
        transferred = full_mc.copy()
        transferred[6] = 0.05
        full_derivative = np.asarray(
            self.function(state, control, full_mc)
        ).reshape(-1)
        transferred_derivative = np.asarray(
            self.function(state, control, transferred)
        ).reshape(-1)
        self.assertNotAlmostEqual(
            full_derivative[11], transferred_derivative[11], places=5
        )


if __name__ == "__main__":
    unittest.main()
