"""Safety-layer output functions shared by live and offline VTOL tests."""

from __future__ import annotations

import math

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


def expected_px4_pusher_assist(
    forward_acceleration: float,
    gravity: float = 9.80665,
    pitch_min_degrees: float = -5.0,
    thrust_scale: float = 0.7,
) -> float:
    """Mirror PX4's MC pusher-assist law for a level forward acceleration.

    This is a prediction/check only. The ROS node does not send this value to
    an actuator; PX4 computes and applies the real pusher command.
    """
    pitch_setpoint = -math.atan2(max(0.0, forward_acceleration), gravity)
    pitch_min = math.radians(pitch_min_degrees)
    if pitch_setpoint >= pitch_min:
        return 0.0
    command = (math.sin(pitch_min) - math.sin(pitch_setpoint)) * thrust_scale
    return float(np.clip(command, 0.0, 0.9))
