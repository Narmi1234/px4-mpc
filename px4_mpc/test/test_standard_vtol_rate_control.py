import unittest

import numpy as np

from px4_mpc.models.standard_vtol_rate_control import StandardVtolRateControlModel


class TestStandardVtolRateControlModel(unittest.TestCase):
    def setUp(self):
        self.model = StandardVtolRateControlModel()

    def test_mc_matches_px4_parallel_pid_equation(self):
        value = self.model.mc_torque(
            rate=np.array([0.1, -0.2, 0.3]),
            rate_setpoint=np.array([0.4, 0.2, -0.1]),
            angular_acceleration=np.array([1.0, -2.0, 3.0]),
            integrator=np.array([0.01, 0.02, -0.03]),
        )
        expected = (
            np.array([0.30, 0.15, 0.20]) * np.array([0.3, 0.4, -0.4])
            + np.array([0.01, 0.02, -0.03])
            - np.array([0.003, 0.003, 0.0]) * np.array([1.0, -2.0, 3.0])
        )
        np.testing.assert_allclose(value, expected)

    def test_fw_airspeed_scaling_and_saturation(self):
        self.assertAlmostEqual(self.model.airspeed_scaling(15.0), 1.0)
        self.assertAlmostEqual(self.model.airspeed_scaling(5.0), 15.0 / 7.0)
        value = self.model.fw_torque(
            rate=np.zeros(3),
            rate_setpoint=np.array([10.0, 10.0, 10.0]),
            angular_acceleration=np.zeros(3),
            integrator=np.zeros(3),
            calibrated_airspeed=15.0,
            compression_gain=np.ones(3),
        )
        np.testing.assert_allclose(value, np.ones(3))


if __name__ == "__main__":
    unittest.main()
