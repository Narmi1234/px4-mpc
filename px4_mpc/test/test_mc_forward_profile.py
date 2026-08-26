"""Tests for the bounded multicopter forward experiment profile."""

import unittest

import numpy as np

from px4_mpc.models.mc_forward_profile import (
    mc_forward_reference_state,
    McForwardProfile,
    pusher_forward_feedforward,
    pusher_forward_reference_state,
    pusher_forward_speed_reference_state,
)
from px4_mpc.models.standard_vtol_gz_model import StandardVtolGazeboModel


class TestMcForwardProfile(unittest.TestCase):
    def setUp(self):
        self.profile = McForwardProfile(
            target_speed=2.0,
            acceleration=1.0,
            hold_seconds=1.0,
        )

    def test_timing_and_distance(self):
        self.assertAlmostEqual(self.profile.acceleration_seconds, np.pi)
        self.assertAlmostEqual(self.profile.motion_seconds, 2.0 * np.pi + 1.0)
        self.assertAlmostEqual(self.profile.profile_seconds, 2.0 * np.pi + 3.0)
        self.assertAlmostEqual(self.profile.final_distance, 2.0 * np.pi + 2.0)

    def test_profile_boundaries_are_continuous(self):
        ramp = self.profile.acceleration_seconds
        final_distance = self.profile.final_distance
        expected = (
            (0.0, 0.0, 0.0, "initial_hover"),
            (2.0, 0.0, 0.0, "accelerate"),
            (2.0 + ramp, ramp, 2.0, "hold_speed"),
            (3.0 + ramp, ramp + 2.0, 2.0, "brake"),
            (3.0 + 2.0 * ramp, final_distance, 0.0, "settle_hover"),
            (20.0, final_distance, 0.0, "settle_hover"),
        )
        for time_seconds, distance, speed, phase in expected:
            with self.subTest(time_seconds=time_seconds):
                sample = self.profile.sample(time_seconds)
                self.assertAlmostEqual(sample.distance, distance)
                self.assertAlmostEqual(sample.speed, speed)
                self.assertEqual(sample.phase, phase)

    def test_invalid_profile_is_rejected(self):
        with self.assertRaises(ValueError):
            McForwardProfile(target_speed=0.0)
        with self.assertRaises(ValueError):
            McForwardProfile(acceleration=-1.0)
        with self.assertRaises(ValueError):
            McForwardProfile(hold_seconds=-0.1)
        with self.assertRaises(ValueError):
            McForwardProfile(start_delay_seconds=-0.1)

    def test_gate_b1_profile_duration_and_distance(self):
        profile = McForwardProfile(
            target_speed=5.0,
            acceleration=0.40,
            hold_seconds=3.0,
            start_delay_seconds=2.0,
        )
        self.assertAlmostEqual(profile.acceleration_seconds, 6.25 * np.pi)
        self.assertAlmostEqual(profile.profile_seconds, 5.0 + 12.5 * np.pi)
        self.assertAlmostEqual(profile.final_distance, 15.0 + 31.25 * np.pi)
        self.assertLess(profile.profile_seconds + 0.5 + 4.0, 49.0)

    def test_reference_follows_heading_and_feedforward_pitch(self):
        hold = np.zeros(10)
        hold[6:10] = [np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)]
        sample = self.profile.sample(3.0)
        reference = mc_forward_reference_state(
            hold, np.array([0.0, 1.0]), sample, gravity=9.80665
        )
        self.assertAlmostEqual(reference[0], 0.0)
        self.assertGreater(reference[1], 0.0)
        self.assertAlmostEqual(reference[3], 0.0)
        self.assertGreater(reference[4], 0.0)
        self.assertAlmostEqual(np.linalg.norm(reference[6:10]), 1.0)

    def test_pusher_reference_moves_without_feedforward_pitch(self):
        hold = np.zeros(10)
        hold[6:10] = [np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)]
        sample = self.profile.sample(3.0)
        reference = pusher_forward_reference_state(
            hold, np.array([0.0, 1.0]), sample
        )
        self.assertGreater(reference[1], 0.0)
        self.assertGreater(reference[4], 0.0)
        np.testing.assert_allclose(reference[6:10], hold[6:10])

    def test_pusher_speed_reference_anchors_only_along_track_to_current_state(self):
        hold = np.zeros(10)
        hold[6] = 1.0
        current = hold.copy()
        current[0:3] = [10.0, 2.0, 3.0]
        direction = np.array([1.0, 0.0])
        base_sample = self.profile.sample(3.0)
        future_sample = self.profile.sample(3.5)

        current_reference = pusher_forward_speed_reference_state(
            hold, current, direction, base_sample, base_sample
        )
        future_reference = pusher_forward_speed_reference_state(
            hold, current, direction, base_sample, future_sample
        )

        self.assertAlmostEqual(current_reference[0], current[0])
        self.assertAlmostEqual(current_reference[1], hold[1])
        self.assertAlmostEqual(current_reference[2], hold[2])
        self.assertAlmostEqual(
            future_reference[0] - current_reference[0],
            future_sample.distance - base_sample.distance,
        )
        self.assertAlmostEqual(future_reference[1], hold[1])
        self.assertAlmostEqual(future_reference[3], future_sample.speed)
        self.assertAlmostEqual(future_reference[4], 0.0)
        np.testing.assert_allclose(future_reference[6:10], hold[6:10])

    def test_pusher_feedforward_is_bounded_and_reduces_for_braking(self):
        plant = StandardVtolGazeboModel()
        accelerate = pusher_forward_feedforward(plant, 2.0, 0.75)
        hold = pusher_forward_feedforward(plant, 2.0, 0.0)
        brake = pusher_forward_feedforward(plant, 2.0, -0.75)
        self.assertAlmostEqual(accelerate, 0.10)
        self.assertGreater(hold, brake)
        self.assertGreaterEqual(brake, 0.0)


if __name__ == "__main__":
    unittest.main()
