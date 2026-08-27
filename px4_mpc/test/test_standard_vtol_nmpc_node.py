import numpy as np

from px4_mpc.standard_vtol_nmpc_node import (
    _state_age_limits,
    _status_tracking_values,
)


def test_status_before_hover_reference_does_not_access_missing_hold_state():
    state = np.zeros(10)
    reference = np.zeros(10)

    tracking_error, displacement = _status_tracking_values(
        state, reference, hold_state=None
    )

    assert tracking_error == [0.0] * 6
    assert displacement == "none"


def test_pretransition_allows_only_the_measured_bounded_dds_gap():
    assert _state_age_limits(False, "pretransition_5mps") == (0.20, 0.20)
    assert _state_age_limits(True, "pusher_forward") == (0.30, 0.20)
    assert _state_age_limits(True, "pretransition_5mps") == (0.45, 0.45)


def test_gate_d_relaxes_dds_gap_only_before_front_transition():
    assert _state_age_limits(
        True, "transition_gate_d", "mc_accelerate"
    ) == (0.45, 0.45)
    assert _state_age_limits(
        True, "transition_gate_d", "front_transition"
    ) == (0.30, 0.20)
