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
            self.model.hover_state(), self.model.hover_control(), np.zeros(6)
        )).reshape(-1)
        self.assertEqual(value.shape, (16,))
        self.assertTrue(np.all(np.isfinite(value)))
        self.assertLess(np.linalg.norm(value[3:6]), 0.1)

    def test_pitch_disturbance_has_frd_to_flu_sign(self):
        positive_frd = np.zeros(6)
        positive_frd[5] = 0.2
        derivative = np.asarray(self.function(
            self.model.hover_state(), self.model.hover_control(), positive_frd
        )).reshape(-1)
        self.assertAlmostEqual(derivative[11], -0.2, places=9)

    def test_lambda_zero_removes_vertical_motor_lift(self):
        state = self.model.hover_state()
        control = self.model.hover_control()
        full = np.asarray(self.function(state, control, np.zeros(6))).reshape(-1)
        control[5] = 0.0
        no_lift = np.asarray(self.function(state, control, np.zeros(6))).reshape(-1)
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
        values = jacobian(x, self.model.hover_control(), np.zeros(6))
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


if __name__ == "__main__":
    unittest.main()
