"""Tests for PX4-time profile scheduling and wall-time diagnostics."""

import math
import unittest

from px4_mpc.models.px4_timebase import Px4Timebase


class TestPx4Timebase(unittest.TestCase):
    def test_slow_simulation_uses_px4_elapsed_for_profile(self):
        timebase = Px4Timebase()
        timebase.update_px4_timestamp(1_000_000)
        timebase.start_offboard(1_000_000, 10_000_000_000)

        # Twelve wall seconds at RTF 0.8 represent only 9.6 plant seconds.
        timebase.update_px4_timestamp(10_600_000)
        self.assertAlmostEqual(timebase.px4_elapsed(), 9.6)
        self.assertAlmostEqual(timebase.wall_elapsed(22_000_000_000), 12.0)
        self.assertAlmostEqual(timebase.realtime_factor(22_000_000_000), 0.8)

    def test_nav_transition_timestamp_is_used_instead_of_callback_time(self):
        timebase = Px4Timebase()
        timebase.update_px4_timestamp(8_500_000)
        timebase.start_offboard(8_000_000, 100)
        self.assertAlmostEqual(timebase.px4_elapsed(), 0.5)

    def test_stop_retains_final_durations(self):
        timebase = Px4Timebase()
        timebase.update_px4_timestamp(2_000_000)
        timebase.start_offboard(2_000_000, 3_000_000_000)
        timebase.update_px4_timestamp(5_250_000)
        px4_elapsed, wall_elapsed = timebase.stop_offboard(7_000_000_000)
        self.assertAlmostEqual(px4_elapsed, 3.25)
        self.assertAlmostEqual(wall_elapsed, 4.0)
        self.assertFalse(timebase.active)
        self.assertAlmostEqual(timebase.px4_elapsed(), 3.25)
        self.assertTrue(math.isfinite(timebase.realtime_factor(9_000_000_000)))


if __name__ == "__main__":
    unittest.main()
