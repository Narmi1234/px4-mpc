"""Tests for PX4/Gazebo coordinate conversion."""

import unittest

import numpy as np

from px4_mpc.models.frames import (
    enu_to_ned,
    flu_to_frd,
    frd_to_flu,
    ned_to_enu,
    px4_quaternion_to_gazebo,
)
from px4_mpc.models.standard_vtol_gz_model import quaternion_to_rotation


class TestFrames(unittest.TestCase):
    def test_world_vector_round_trip(self):
        vector = np.array([1.0, -2.0, 3.0])
        np.testing.assert_allclose(enu_to_ned(ned_to_enu(vector)), vector)

    def test_body_vector_round_trip(self):
        vector = np.array([1.0, -2.0, 3.0])
        np.testing.assert_allclose(flu_to_frd(frd_to_flu(vector)), vector)

    def test_level_north_facing_attitude(self):
        q_enu_flu = px4_quaternion_to_gazebo(np.array([1.0, 0.0, 0.0, 0.0]))
        rotation = quaternion_to_rotation(q_enu_flu)
        # PX4 body x pointing North becomes Gazebo body x pointing ENU +y.
        np.testing.assert_allclose(rotation[:, 0], [0.0, 1.0, 0.0], atol=1.0e-12)
        # PX4 FRD down and Gazebo FLU up both map consistently to world vertical.
        np.testing.assert_allclose(rotation[:, 2], [0.0, 0.0, 1.0], atol=1.0e-12)


if __name__ == "__main__":
    unittest.main()
