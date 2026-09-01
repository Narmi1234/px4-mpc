from types import MethodType, SimpleNamespace
import unittest

import numpy as np

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
            l3_hold_seconds=4.0,
            l3_min_lambda=0.35,
            l3_pusher_max=0.42,
            max_state_age=0.20,
            active_state_stale_abort=0.45,
        )
        self.node._allocation_configuration = MethodType(
            StandardVtolRobustShadow._allocation_configuration,
            self.node,
        )
        self.node.controller = SimpleNamespace(N=20)

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


if __name__ == "__main__":
    unittest.main()
