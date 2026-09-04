from types import MethodType, SimpleNamespace
import unittest

import numpy as np

from px4_mpc.models.standard_vtol_robust_casadi_model import (
    StandardVtolRobustCasadiModel,
)
from px4_mpc.standard_vtol_robust_shadow_node import StandardVtolRobustShadow


class TestStandardVtolL1Profile(unittest.TestCase):
    def setUp(self):
        self.node = SimpleNamespace(
            test_mode="allocation_l1",
            l1_target_speed=5.0,
            l1_acceleration=0.4,
            l1_hold_seconds=3.0,
            l1_min_lambda=0.8,
            l1_pusher_max=0.25,
            l2_target_speed=9.0,
            l2_acceleration=0.4,
            l2_hold_seconds=3.0,
            l2_min_lambda=0.5,
            l2_pusher_max=0.35,
            l3_target_speed=10.5,
            l3_acceleration=0.30,
            l3_brake_rate=0.50,
            l3_recovery_seconds=0.0,
            l3_brake_entry_lambda=0.70,
            l3_hold_seconds=4.0,
            l3_min_lambda=0.35,
            l3_pusher_max=0.42,
            l3_collective_min=0.30,
            l3_effective_lift_min=0.0,
            l3_pitch_rate_limit=0.18,
            l3_pitch_damping_gain=0.0,
            l3_vertical_correction_gain=1.0,
            l3_vertical_speed_limit=0.9,
            l3_vertical_speed_persistence=0.2,
            l3_vertical_speed_emergency_limit=1.2,
            vertical_speed_violation_since_ns=0,
            max_state_age=0.20,
            active_state_stale_abort=0.45,
            lambda_prediction_slew_rate=0.05,
        )
        self.node._allocation_configuration = MethodType(
            StandardVtolRobustShadow._allocation_configuration,
            self.node,
        )
        self.node.controller = SimpleNamespace(N=20, dt=0.1)

    def speed(self, elapsed):
        return StandardVtolRobustShadow._l1_speed_reference(
            self.node, elapsed
        )

    def test_guarded_accelerate_hold_brake_profile(self):
        self.assertEqual(self.speed(0.5), 0.0)
        self.assertAlmostEqual(self.speed(6.0), 2.0)
        self.assertAlmostEqual(self.speed(13.5), 5.0)
        self.assertAlmostEqual(self.speed(16.0), 5.0)
        self.assertAlmostEqual(self.speed(21.5), 2.5)
        self.assertEqual(self.speed(27.0), 0.0)

    def test_total_includes_three_second_settle(self):
        duration = StandardVtolRobustShadow._l1_total_seconds(self.node)
        self.assertAlmostEqual(duration, 29.5)

    def test_l2_profile_and_four_second_settle(self):
        self.node.test_mode = "allocation_l2"
        self.assertAlmostEqual(self.speed(11.0), 4.0)
        self.assertAlmostEqual(self.speed(23.5), 9.0)
        self.assertAlmostEqual(self.speed(28.0), 8.25)
        self.assertAlmostEqual(self.speed(44.5), 0.0)
        duration = StandardVtolRobustShadow._l1_total_seconds(self.node)
        self.assertAlmostEqual(duration, 48.5)

    def test_l2_pitch_corridor_is_bounded(self):
        pitch = StandardVtolRobustShadow._l2_pitch_reference
        values = [pitch(speed, 9.0) for speed in range(10)]
        self.assertAlmostEqual(values[0], 0.0)
        self.assertLess(min(values), 0.0)
        self.assertGreaterEqual(min(values), -0.15)

    def test_state_freshness_has_bounded_hold_window(self):
        action = StandardVtolRobustShadow._state_freshness_action
        self.assertEqual(action(self.node, 0.199), "solve")
        self.assertEqual(action(self.node, 0.200), "solve")
        self.assertEqual(action(self.node, 0.201), "hold")
        self.assertEqual(action(self.node, 0.449), "hold")
        self.assertEqual(action(self.node, 0.450), "abort")

    def test_l2_allocation_bounds_follow_reference_with_freedom(self):
        self.node.test_mode = "allocation_l2"
        references = np.zeros((20, 6))
        references[:, 5] = np.linspace(1.0, 0.5, 20)
        lower, upper = StandardVtolRobustShadow._allocation_control_bounds(
            self.node, references
        )
        self.assertAlmostEqual(lower[0, 5], 0.95)
        self.assertAlmostEqual(upper[0, 5], 1.0)
        self.assertAlmostEqual(upper[-1, 5], 0.55)

    def test_l3_profile_and_allocation_bounds(self):
        self.node.test_mode = "allocation_l3"
        target, acceleration, hold, minimum, pusher = (
            StandardVtolRobustShadow._allocation_configuration(self.node)
        )
        self.assertEqual((target, acceleration, hold, minimum, pusher),
                         (10.5, 0.30, 4.0, 0.35, 0.42))
        references = np.zeros((20, 6))
        references[:, 5] = np.linspace(1.0, 0.35, 20)
        lower, upper = StandardVtolRobustShadow._allocation_control_bounds(
            self.node, references
        )
        self.assertAlmostEqual(lower[-1, 5], 0.35)
        self.assertAlmostEqual(upper[-1, 5], 0.40)

    def test_allocation_prediction_is_anchored_to_applied_slew(self):
        self.node.test_mode = "allocation_l3"
        references = np.zeros((20, 6))
        references[:, 5] = 0.70
        lower, upper = StandardVtolRobustShadow._allocation_control_bounds(
            self.node, references, applied_lambda=0.39
        )
        self.assertAlmostEqual(lower[0, 5], 0.395)
        self.assertAlmostEqual(upper[0, 5], 0.395)
        self.assertLessEqual(upper[9, 5], 0.44 + 1.0e-12)
        self.assertLessEqual(upper[-1, 5], 0.49 + 1.0e-12)

    def test_elevator_feedforward_is_zero_in_mc_and_hard_limited(self):
        command = StandardVtolRobustCasadiModel.elevator_feedforward
        self.assertEqual(command(10.5, 1.0), 0.0)
        self.assertGreater(command(10.5, 0.8), 0.0)
        self.assertAlmostEqual(command(10.5, 0.35), 0.25)

    def test_l3_vertical_guard_filters_spike_but_rejects_persistence(self):
        guard = StandardVtolRobustShadow._vertical_speed_safety_reason
        self.assertIsNone(guard(self.node, 0.95, 0.9, True, 1_000_000_000))
        self.assertIsNone(guard(self.node, 0.95, 0.9, True, 1_150_000_000))
        self.assertEqual(
            guard(self.node, 0.95, 0.9, True, 1_200_000_000),
            "vertical_speed_limit",
        )
        self.assertIsNone(guard(self.node, 0.2, 0.9, True, 1_250_000_000))
        self.assertEqual(self.node.vertical_speed_violation_since_ns, 0)

    def test_l3_vertical_guard_keeps_immediate_emergency_abort(self):
        guard = StandardVtolRobustShadow._vertical_speed_safety_reason
        self.assertEqual(
            guard(self.node, -1.21, 0.9, True, 1_000_000_000),
            "vertical_speed_emergency_limit",
        )

    def test_non_l3_vertical_guard_remains_immediate(self):
        guard = StandardVtolRobustShadow._vertical_speed_safety_reason
        self.assertEqual(
            guard(self.node, 0.81, 0.8, False, 1_000_000_000),
            "vertical_speed_limit",
        )

    def test_pitch_damping_grows_as_lift_allocation_decreases(self):
        damp = StandardVtolRobustShadow._damped_pitch_rate_command
        self.assertAlmostEqual(damp(-0.05, -0.16, 1.0, 0.75, 0.25), -0.05)
        self.assertGreater(damp(-0.05, -0.16, 0.25, 0.75, 0.25), 0.0)
        self.assertAlmostEqual(damp(0.2, -1.0, 0.2, 1.0, 0.25), 0.25)

    def test_effective_lift_floor_compensates_allocation_and_is_bounded(self):
        floor = StandardVtolRobustShadow._minimum_collective_for_allocation
        self.assertAlmostEqual(floor(0.46, 0.20, 0.50, 12.0), 0.40)
        self.assertAlmostEqual(floor(0.46, 0.20, 0.40, 12.0), 0.50)
        self.assertAlmostEqual(floor(0.46, 0.20, 0.30, 12.0), 2.0 / 3.0)
        self.assertAlmostEqual(floor(0.46, 0.20, 0.20, 12.0), 0.70)
        self.assertAlmostEqual(floor(0.46, 0.20, 1.0, 0.0), 0.520119535)
        self.assertAlmostEqual(floor(0.46, 0.0, 0.30, 12.0), 0.46)

    def test_l4a_separates_roll_pitch_transfer_from_lift_and_keeps_bounds(self):
        weight = StandardVtolRobustShadow._l4a_roll_pitch_weight
        self.assertAlmostEqual(weight(1.0, 0.2, 0.05), 1.0)
        self.assertAlmostEqual(weight(0.6, 0.2, 0.05), 0.525)
        self.assertAlmostEqual(weight(0.2, 0.2, 0.05), 0.05)
        self.assertAlmostEqual(weight(0.0, 0.2, 0.05), 0.05)

    def test_l4b_lateral_reference_is_smooth_and_returns_to_track(self):
        self.node.l3_target_speed = 12.0
        self.node.l3_acceleration = 0.25
        reference = StandardVtolRobustShadow._l4b_lateral_reference
        start = 1.0 + 12.0 / 0.25

        self.assertEqual(reference(self.node, start), (0.0, 0.0))
        position, speed = reference(self.node, start + 3.0)
        self.assertAlmostEqual(position, 0.375)
        self.assertAlmostEqual(speed, 0.75 * np.pi / 12.0)
        position, speed = reference(self.node, start + 6.0)
        self.assertAlmostEqual(position, 0.75)
        self.assertAlmostEqual(speed, 0.0)
        position, speed = reference(self.node, start + 9.0)
        self.assertAlmostEqual(position, 0.375)
        self.assertAlmostEqual(speed, -0.75 * np.pi / 12.0)
        self.assertEqual(reference(self.node, start + 12.0), (0.0, 0.0))

    def test_l4c_bounds_force_predicted_motor_off_when_reachable(self):
        self.node.test_mode = "allocation_l4c"
        self.node.l3_min_lambda = 0.0
        references = np.zeros((20, 6))
        lower, upper = StandardVtolRobustShadow._allocation_control_bounds(
            self.node, references, applied_lambda=0.0
        )
        np.testing.assert_allclose(lower[:, 5], 0.0)
        np.testing.assert_allclose(upper[:, 5], 0.0)

    def test_l4c_pitch_uses_independent_12mps_ulog_trim(self):
        pitch = StandardVtolRobustShadow._l4c_pitch_reference
        self.assertAlmostEqual(np.degrees(pitch(12.0, 12.0)), -4.10)
        self.assertAlmostEqual(np.degrees(pitch(0.0, 12.0)), 0.0)

    def test_l4c_airspeed_interlock_prevents_early_motor_off(self):
        floor = StandardVtolRobustShadow._airspeed_allocation_floor
        self.assertEqual(floor(float("nan"), 4.0, 12.0), 1.0)
        self.assertEqual(floor(-1.0, 4.0, 12.0), 1.0)
        self.assertEqual(floor(4.0, 4.0, 12.0), 1.0)
        self.assertAlmostEqual(floor(8.0, 4.0, 12.0), 0.5)
        self.assertAlmostEqual(floor(11.6, 4.0, 12.0), 0.05)
        self.assertEqual(floor(12.0, 4.0, 12.0), 0.0)
        self.assertEqual(floor(15.0, 4.0, 12.0), 0.0)

    def test_l4c_collective_handover_is_bumpless_above_low_cas(self):
        blend = StandardVtolRobustShadow._l4c_nmpc_collective_blend
        self.assertEqual(blend(float("nan"), 4.0), 0.0)
        self.assertEqual(blend(-1.0, 4.0), 0.0)
        self.assertEqual(blend(4.0, 4.0), 0.0)
        self.assertAlmostEqual(blend(5.0, 4.0), 0.5)
        self.assertEqual(blend(6.0, 4.0), 1.0)
        self.assertEqual(blend(12.0, 4.0), 1.0)


if __name__ == "__main__":
    unittest.main()
