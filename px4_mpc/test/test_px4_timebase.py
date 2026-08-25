"""Tests for PX4-time profile scheduling and wall-time diagnostics."""

import math
import unittest

from px4_mpc.models.px4_timebase import Px4Timebase


class TestPx4Timebase(unittest.TestCase):
    def test_changing_dds_offset_recovers_monotonic_px4_boot_time(self):
        timebase = Px4Timebase()
        translated_start = 1_787_292_400_000_000
        raw_start = 100_000_000
        timebase.update_timesync(
            translated_start,
            raw_start - translated_start,
            translated_start,
            raw_start - translated_start,
            wall_ns=10_000_000_000,
        )
        timebase.start_offboard(raw_start, 10_000_000_000)

        # The translated clock advances three seconds while PX4 advances one;
        # the changed timesync offset removes the extra two seconds.
        translated_next = translated_start + 3_000_000
        raw_next = raw_start + 1_000_000
        timebase.update_timesync(
            translated_next,
            raw_next - translated_next,
            translated_next,
            raw_next - translated_next,
            wall_ns=11_000_000_000,
        )

        self.assertTrue(timebase.synchronized)
        self.assertEqual(timebase.latest_px4_us, raw_next)
        self.assertAlmostEqual(timebase.px4_elapsed(), 1.0)

    def test_direct_timesync_interpolates_at_measured_simulation_rate(self):
        timebase = Px4Timebase()
        translated = 1_787_292_400_000_000
        raw = 100_000_000
        timebase.update_timesync(
            translated,
            raw - translated,
            translated,
            raw - translated,
            wall_ns=10_000_000_000,
        )
        timebase.update_timesync(
            translated + 1_000_000,
            raw + 800_000 - (translated + 1_000_000),
            translated + 1_000_000,
            raw + 800_000 - (translated + 1_000_000),
            wall_ns=11_000_000_000,
        )

        timebase.advance_from_wall(12_000_000_000)

        self.assertAlmostEqual(timebase.px4_rate_per_wall, 0.8)
        self.assertEqual(timebase.latest_px4_us, raw + 1_600_000)

    def test_offboard_start_reanchors_interpolated_clock(self):
        timebase = Px4Timebase(
            latest_px4_us=101_000_000,
            sync_anchor_px4_us=100_000_000,
            sync_anchor_wall_ns=10_000_000_000,
            px4_rate_per_wall=0.8,
        )
        timebase.start_offboard(100_900_000, 11_000_000_000)
        timebase.advance_from_wall(12_000_000_000)

        self.assertEqual(timebase.latest_px4_us, 101_700_000)
        self.assertAlmostEqual(timebase.px4_elapsed(), 0.8)

    def test_timesync_anchor_corrects_transient_forward_offset_jump(self):
        timebase = Px4Timebase()
        translated = 1_787_634_100_000_000
        raw = 150_000_000
        old_offset = raw - translated
        timebase.update_timesync(
            translated,
            old_offset,
            translated,
            old_offset,
        )

        # DDS starts serializing with a new offset before its status callback
        # reaches this node. The resulting 1.3 s leap must not be accepted.
        timebase.update_translated_timestamp(translated + 1_350_000)
        self.assertEqual(timebase.latest_px4_us, raw)

        next_raw = raw + 1_000_000
        next_remote = translated + 1_150_000
        next_observed_offset = next_raw - next_remote
        timebase.update_timesync(
            next_remote,
            next_observed_offset,
            next_remote,
            next_observed_offset,
        )
        self.assertEqual(timebase.latest_px4_us, next_raw)

    def test_offboard_start_discards_preflight_clock_bias(self):
        timebase = Px4Timebase(latest_px4_us=101_300_000)
        timebase.start_offboard(100_000_000, 10_000_000_000)
        self.assertEqual(timebase.latest_px4_us, 100_000_000)
        self.assertAlmostEqual(timebase.px4_elapsed(), 0.0)

    def test_slow_simulation_uses_px4_elapsed_for_profile(self):
        timebase = Px4Timebase()
        timebase.update_px4_timestamp(1_000_000)
        timebase.start_offboard(1_000_000, 10_000_000_000)

        # Twelve wall seconds at RTF 0.8 represent only 9.6 plant seconds.
        timebase.update_px4_timestamp(10_600_000)
        self.assertAlmostEqual(timebase.px4_elapsed(), 9.6)
        self.assertAlmostEqual(timebase.wall_elapsed(22_000_000_000), 12.0)
        self.assertAlmostEqual(timebase.realtime_factor(22_000_000_000), 0.8)

    def test_offboard_event_timestamp_defines_interval_zero(self):
        timebase = Px4Timebase()
        timebase.update_px4_timestamp(8_500_000)
        timebase.start_offboard(8_000_000, 100)
        self.assertEqual(timebase.latest_px4_us, 8_000_000)
        self.assertAlmostEqual(timebase.px4_elapsed(), 0.0)

    def test_mixed_timestamp_domain_falls_back_to_latest_px4_clock(self):
        timebase = Px4Timebase()
        epoch_timestamp = 1_787_207_600_000_000
        boot_relative_timestamp = 224_348_000
        timebase.update_px4_timestamp(epoch_timestamp)

        timebase.start_offboard(boot_relative_timestamp, 100)

        self.assertEqual(timebase.offboard_start_px4_us, epoch_timestamp)
        self.assertAlmostEqual(timebase.px4_elapsed(), 0.0)
        timebase.update_px4_timestamp(epoch_timestamp + 500_000)
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
