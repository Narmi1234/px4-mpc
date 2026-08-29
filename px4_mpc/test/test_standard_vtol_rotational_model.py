import unittest

import numpy as np

from px4_mpc.models.standard_vtol_rotational_model import StandardVtolTorqueInformedModel


class TestStandardVtolTorqueInformedModel(unittest.TestCase):
    def setUp(self):
        self.model = StandardVtolTorqueInformedModel()

    def test_dimensions_and_finite_hover_derivative(self):
        state = self.model.hover_state()
        control = self.model.hover_control()
        derivative = self.model.derivative(state, control)
        self.assertEqual(derivative.shape, (16,))
        self.assertTrue(np.all(np.isfinite(derivative)))
        self.assertLess(np.linalg.norm(derivative[3:6]), 0.1)

    def test_lambda_endpoints_select_actuator_group(self):
        state = self.model.hover_state()
        control = self.model.hover_control()
        control[3] = -0.2  # FLU pitch-rate command = positive FRD pitch
        motor_mc, surfaces_mc, _, _ = self.model.actuator_commands(state, control)
        self.assertGreater(np.mean(motor_mc[:4]), 0.4)
        np.testing.assert_allclose(surfaces_mc, 0.0, atol=1e-12)

        control[5] = 0.0
        motor_fw, surfaces_fw, _, _ = self.model.actuator_commands(state, control)
        self.assertLess(np.max(motor_fw[:4]), 0.01)
        self.assertGreater(abs(surfaces_fw[2]), 0.01)

    def test_invalid_shapes_are_rejected(self):
        with self.assertRaises(ValueError):
            self.model.actuator_commands(np.zeros(12), self.model.hover_control())
        with self.assertRaises(ValueError):
            self.model.actuator_commands(self.model.hover_state(), np.zeros(5))

    def test_surface_state_has_slow_first_order_response(self):
        command = np.array([0.1, -0.1, 0.2])
        initial = np.zeros(3)
        after_20_ms = self.model.step_surface_state(initial, command, 0.02)
        target = self.model.surface_static_gains * command
        self.assertTrue(np.all(np.abs(after_20_ms) < np.abs(target)))
        after_20_s = self.model.step_surface_state(initial, command, 20.0)
        np.testing.assert_allclose(after_20_s, target, atol=1e-6)

        state = self.model.hover_state()
        control = self.model.hover_control()
        control[3] = -0.2
        control[5] = 0.0
        derivative = self.model.derivative(state, control)
        self.assertGreater(abs(derivative[15]), 0.01)

    def test_identified_pitch_moment_is_finite_and_elevator_sensitive(self):
        velocity = np.array([12.0, 0.0, 0.0])
        omega = np.zeros(3)
        _, neutral = self.model.aerodynamic_wrench(np.zeros(3), velocity, omega)
        _, deflected = self.model.aerodynamic_wrench(
            np.array([0.0, 0.0, 0.2]), velocity, omega
        )
        self.assertTrue(np.all(np.isfinite(neutral)))
        self.assertLess(deflected[1], neutral[1])


if __name__ == "__main__":
    unittest.main()
