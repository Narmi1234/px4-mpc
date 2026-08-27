"""Tests for the guarded Gate D transition state machine."""

import unittest

from px4_mpc.models.transition_gate_d import (
    GateDStateMachine,
    px4_mc_weight,
    transition_pitch_and_elevator,
    transition_pusher_trim,
    VTOL_FW,
    VTOL_MC,
    VTOL_TRANSITION_TO_FW,
    VTOL_TRANSITION_TO_MC,
)


class TestGateDStateMachine(unittest.TestCase):
    def test_complete_front_and_back_sequence(self):
        gate = GateDStateMachine()
        gate.start(0.0)
        self.assertEqual(gate.update(10.0, VTOL_MC, 7.6, 7.6).state, "mc_accelerate")
        update = gate.update(11.1, VTOL_MC, 7.7, 7.7)
        self.assertEqual(update.transition_request, VTOL_FW)
        self.assertEqual(gate.update(12.0, VTOL_TRANSITION_TO_FW, 9.0, 9.0).state, "front_transition")
        self.assertEqual(gate.update(14.0, VTOL_FW, 11.0, 11.0).state, "fw_hold")
        update = gate.update(19.1, VTOL_FW, 12.0, 12.0)
        self.assertEqual(update.transition_request, VTOL_MC)
        self.assertEqual(gate.update(21.0, VTOL_TRANSITION_TO_MC, 9.0, 9.0).state, "back_transition")
        self.assertEqual(gate.update(24.0, VTOL_MC, 5.0, 5.0).state, "mc_recovered")
        gate.update(30.0, VTOL_MC, 0.2, 0.2)
        self.assertTrue(gate.update(32.1, VTOL_MC, 0.1, 0.1).completed)

    def test_front_timeout_requests_mc_recovery(self):
        gate = GateDStateMachine()
        gate.start(0.0)
        gate.update(1.0, VTOL_MC, 8.0, 8.0)
        gate.update(2.1, VTOL_MC, 8.0, 8.0)
        update = gate.update(15.0, VTOL_TRANSITION_TO_FW, 9.0, 9.0)
        self.assertEqual(update.state, "abort_recovery")
        self.assertEqual(update.transition_request, VTOL_MC)

    def test_reference_and_blend_schedules_are_bounded(self):
        gate = GateDStateMachine()
        gate.start(0.0)
        self.assertAlmostEqual(gate.sample(0.0).speed, 0.0)
        self.assertAlmostEqual(gate.sample(100.0).speed, 8.0)
        self.assertAlmostEqual(px4_mc_weight(VTOL_TRANSITION_TO_FW, 9.0, 2.0), 0.5)
        self.assertAlmostEqual(px4_mc_weight(VTOL_TRANSITION_TO_MC, 1.5, 1.5), 0.5)
        pitch, elevator = transition_pitch_and_elevator(10.0, 0.0)
        self.assertLess(pitch, 0.0)
        self.assertGreater(elevator, 0.0)
        pitch, elevator = transition_pitch_and_elevator(
            10.0, 0.0, VTOL_TRANSITION_TO_FW
        )
        self.assertEqual((pitch, elevator), (0.0, 0.0))
        self.assertAlmostEqual(transition_pusher_trim(12.0, 0.0), 0.2660)
        self.assertAlmostEqual(transition_pusher_trim(12.0, 1.0), 0.0)

    def test_reference_is_continuous_and_completion_waits_for_zero_pusher(self):
        gate = GateDStateMachine()
        gate.start(0.0)
        gate.update(20.0, VTOL_MC, 7.6, 7.6)
        before = gate.sample(21.1)
        gate.update(21.1, VTOL_MC, 7.7, 7.7)
        after = gate.sample(21.1)
        self.assertAlmostEqual(after.distance, before.distance)
        self.assertAlmostEqual(after.speed, before.speed)

        gate._enter("mc_recovered", 30.0)
        gate.update(31.0, VTOL_MC, 0.1, 0.1, commanded_pusher=0.02)
        self.assertIsNone(gate.condition_since_s)

    def test_front_profile_uses_separate_stock_informed_pusher_envelope(self):
        gate = GateDStateMachine()
        self.assertAlmostEqual(gate.mc_pusher_limit, 0.30)
        self.assertAlmostEqual(gate.front_pusher_command, 0.60)
        self.assertAlmostEqual(gate.front_peak_acceleration, 2.0)


if __name__ == "__main__":
    unittest.main()
