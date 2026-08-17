"""Tests for the first custom PX4 external-pusher pulse."""

import unittest

from px4_mpc.models.external_pusher_profile import ExternalPusherProfile


class TestExternalPusherProfile(unittest.TestCase):
    def test_exact_phases_and_bounds(self):
        profile = ExternalPusherProfile()
        self.assertAlmostEqual(profile.ramp_seconds, 2.5)
        self.assertAlmostEqual(profile.profile_seconds, 9.0)
        expected = (
            (0.0, 0.0, "initial_hover"),
            (2.0, 0.0, "pusher_ramp_up"),
            (3.25, 0.025, "pusher_ramp_up"),
            (4.5, 0.05, "pusher_hold"),
            (6.5, 0.05, "pusher_ramp_down"),
            (7.75, 0.025, "pusher_ramp_down"),
            (9.0, 0.0, "settle_hover"),
        )
        for time_seconds, command, phase in expected:
            sample = profile.sample(time_seconds)
            self.assertAlmostEqual(sample.command, command)
            self.assertEqual(sample.phase, phase)

    def test_unsafe_peak_is_rejected(self):
        with self.assertRaises(ValueError):
            ExternalPusherProfile(peak_command=0.051)


if __name__ == "__main__":
    unittest.main()
