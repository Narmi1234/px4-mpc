from types import SimpleNamespace
import unittest

from px4_mpc.standard_vtol_robust_shadow_node import StandardVtolRobustShadow


class TestStandardVtolL1Profile(unittest.TestCase):
    def setUp(self):
        self.node = SimpleNamespace(
            l1_target_speed=5.0,
            l1_acceleration=0.4,
            l1_hold_seconds=3.0,
        )

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


if __name__ == "__main__":
    unittest.main()
