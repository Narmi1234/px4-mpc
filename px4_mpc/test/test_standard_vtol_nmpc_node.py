import numpy as np

from px4_mpc.standard_vtol_nmpc_node import _status_tracking_values


def test_status_before_hover_reference_does_not_access_missing_hold_state():
    state = np.zeros(10)
    reference = np.zeros(10)

    tracking_error, displacement = _status_tracking_values(
        state, reference, hold_state=None
    )

    assert tracking_error == [0.0] * 6
    assert displacement == "none"
