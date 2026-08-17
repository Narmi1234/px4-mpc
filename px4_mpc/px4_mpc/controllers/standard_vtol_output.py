"""Safety-layer output functions shared by live and offline VTOL tests."""

from __future__ import annotations

import numpy as np


def limit_mc_command(previous, requested, dt: float = 0.05) -> np.ndarray:
    """Apply the validated lift/rate/slew limits and force pusher to zero."""
    previous = np.asarray(previous, dtype=float)
    requested = np.asarray(requested, dtype=float).copy()
    requested[0] = np.clip(requested[0], 0.48, 0.56)
    requested[1] = 0.0
    requested[2:5] = np.clip(
        0.5 * requested[2:5],
        [-0.20, -0.20, -0.15],
        [0.20, 0.20, 0.15],
    )
    slew_per_second = np.array([0.10, 0.0, 0.30, 0.30, 0.20])
    limited = previous + np.clip(
        requested - previous,
        -slew_per_second * dt,
        slew_per_second * dt,
    )
    limited[1] = 0.0
    return limited


def limit_external_pusher_command(
    previous,
    requested,
    pusher_command: float,
    dt: float = 0.05,
) -> np.ndarray:
    """Apply MC limits while allowing only the first 0.05 pusher pulse."""
    previous = np.asarray(previous, dtype=float)
    limited = limit_mc_command(previous, requested, dt)
    target = float(np.clip(pusher_command, 0.0, 0.05))
    limited[1] = previous[1] + np.clip(
        target - previous[1], -0.02 * dt, 0.02 * dt
    )
    return limited


def vertical_hover_lift(plant, altitude_error: float, vertical_speed: float) -> float:
    """Return the critically damped hover lift command used by the live node."""
    motor = plant.motors[0]
    hover = plant.hover_command
    hover_speed = motor.target_speed(hover)
    acceleration_per_command = (
        8.0
        * motor.motor_constant
        * hover_speed
        * (motor.maximum_speed - motor.minimum_speed)
        / plant.mass
    )
    desired_acceleration = -float(altitude_error) - 2.0 * float(vertical_speed)
    return float(hover + desired_acceleration / acceleration_per_command)
