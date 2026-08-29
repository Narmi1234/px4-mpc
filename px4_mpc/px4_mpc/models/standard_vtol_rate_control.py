"""Reduced NumPy mirror of PX4 Standard VTOL MC/FW rate-control outputs."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


def _array(values) -> np.ndarray:
    return np.asarray(values, dtype=float)


@dataclass(frozen=True)
class RateGains:
    proportional: np.ndarray
    integral: np.ndarray
    derivative: np.ndarray
    feedforward: np.ndarray


@dataclass(frozen=True)
class StandardVtolRateControlParameters:
    """Parameters logged by the PX4 Gazebo Standard VTOL airframe."""

    mc: RateGains = field(default_factory=lambda: RateGains(
        proportional=_array([0.30, 0.15, 0.20]),
        integral=_array([0.20, 0.20, 0.10]),
        derivative=_array([0.003, 0.003, 0.0]),
        feedforward=_array([0.0, 0.0, 0.0]),
    ))
    fw: RateGains = field(default_factory=lambda: RateGains(
        proportional=_array([0.30, 0.90, 0.05]),
        integral=_array([0.10, 0.10, 0.10]),
        derivative=_array([0.0, 0.0, 0.0]),
        feedforward=_array([0.10, 0.20, 0.30]),
    ))
    fw_airspeed_trim: float = 15.0
    fw_airspeed_stall: float = 7.0
    fw_airspeed_scaling_enabled: bool = True
    trim: np.ndarray = field(default_factory=lambda: np.zeros(3))


class StandardVtolRateControlModel:
    """Predict PX4 virtual MC and FW normalized torque setpoints.

    Integrator and FW gain-compression values are supplied as measured
    scheduling parameters for one-step validation. In the future OCP they are
    bounded parameters initialized from PX4, not optimized actuator states.
    """

    def __init__(self, parameters: StandardVtolRateControlParameters | None = None):
        self.parameters = parameters or StandardVtolRateControlParameters()

    @staticmethod
    def pid_output(
        rate: np.ndarray,
        rate_setpoint: np.ndarray,
        angular_acceleration: np.ndarray,
        integrator: np.ndarray,
        gains: RateGains,
        feedforward: np.ndarray | None = None,
    ) -> np.ndarray:
        rate = _array(rate)
        rate_setpoint = _array(rate_setpoint)
        angular_acceleration = _array(angular_acceleration)
        integrator = _array(integrator)
        ff = gains.feedforward if feedforward is None else _array(feedforward)
        return (
            gains.proportional * (rate_setpoint - rate)
            + integrator
            - gains.derivative * angular_acceleration
            + ff * rate_setpoint
        )

    def mc_torque(
        self,
        rate: np.ndarray,
        rate_setpoint: np.ndarray,
        angular_acceleration: np.ndarray,
        integrator: np.ndarray,
    ) -> np.ndarray:
        return self.pid_output(
            rate, rate_setpoint, angular_acceleration, integrator,
            self.parameters.mc,
        )

    def airspeed_scaling(self, calibrated_airspeed: float) -> float:
        if not self.parameters.fw_airspeed_scaling_enabled:
            return 1.0
        constrained = max(float(calibrated_airspeed), self.parameters.fw_airspeed_stall)
        return self.parameters.fw_airspeed_trim / constrained

    def fw_torque(
        self,
        rate: np.ndarray,
        rate_setpoint: np.ndarray,
        angular_acceleration: np.ndarray,
        integrator: np.ndarray,
        calibrated_airspeed: float,
        compression_gain: np.ndarray | None = None,
    ) -> np.ndarray:
        scaling = self.airspeed_scaling(calibrated_airspeed)
        scaled_ff = self.parameters.fw.feedforward / scaling
        raw = self.pid_output(
            rate, rate_setpoint, angular_acceleration, integrator,
            self.parameters.fw, scaled_ff,
        )
        compression = np.ones(3) if compression_gain is None else _array(compression_gain)
        control = compression * raw * scaling * scaling
        trim = self.parameters.trim * scaling * scaling
        return np.clip(control + trim, -1.0, 1.0)
