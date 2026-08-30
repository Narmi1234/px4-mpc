"""Stable LPV pitch-rate dynamics for robust Standard VTOL prediction."""

from __future__ import annotations

import numpy as np


class StandardVtolPitchRateLpvModel:
    """Markov closed-loop pitch model identified on 12 and 15 m/s ULogs.

    ``mu = 1 - lambda`` denotes aerodynamic allocation.  Four nonnegative
    damping and input-gain nodes are bilinearly interpolated over airspeed and
    allocation.  The model is a bounded nominal candidate: robust simulations
    must include ``pitch_disturbance_bound`` rather than treating it as exact.
    """

    damping_nodes = np.array([
        6.073797251922061,
        8.879985149013423,
        0.020000000000000004,
        0.6412218212587313,
    ])
    input_gain_nodes = np.array([
        7.098995985125193,
        4.271124566884213,
        1.2500676781570328e-17,
        0.38489058332832526,
    ])
    transition_bias_coefficients = np.array([
        0.32151702428288986,
        4.119818564996283,
        -3.151421700407417,
    ])
    pitch_disturbance_bound = 0.47  # validation p95 rounded up [rad/s^2]
    speed_upper_node = 20.0

    @classmethod
    def scheduling_basis(cls, airspeed: float, lift_fraction: float) -> np.ndarray:
        speed = float(np.clip(airspeed / cls.speed_upper_node, 0.0, 1.0))
        lift = float(np.clip(lift_fraction, 0.0, 1.0))
        mu = 1.0 - lift
        return np.array([
            (1.0 - speed) * (1.0 - mu),
            speed * (1.0 - mu),
            (1.0 - speed) * mu,
            speed * mu,
        ])

    @classmethod
    def coefficients(
        cls, airspeed: float, lift_fraction: float
    ) -> tuple[float, float]:
        basis = cls.scheduling_basis(airspeed, lift_fraction)
        return (
            float(basis @ cls.damping_nodes),
            float(basis @ cls.input_gain_nodes),
        )

    @classmethod
    def derivative(
        cls,
        pitch_rate: float,
        pitch_rate_setpoint: float,
        airspeed: float,
        lift_fraction: float,
        angle_of_attack: float,
        pitch: float,
        disturbance: float = 0.0,
    ) -> float:
        damping, input_gain = cls.coefficients(airspeed, lift_fraction)
        lift = float(np.clip(lift_fraction, 0.0, 1.0))
        mu = 1.0 - lift
        transition = 4.0 * mu * (1.0 - mu)
        bias_features = np.array([1.0, angle_of_attack, pitch])
        bias = transition * float(bias_features @ cls.transition_bias_coefficients)
        bounded = float(np.clip(
            disturbance, -cls.pitch_disturbance_bound, cls.pitch_disturbance_bound
        ))
        return (
            -damping * float(pitch_rate)
            + input_gain * float(pitch_rate_setpoint)
            + bias
            + bounded
        )

