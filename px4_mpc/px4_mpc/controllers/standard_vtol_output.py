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


def limit_pusher_forward_command(
    previous,
    requested,
    dt: float = 0.05,
) -> np.ndarray:
    """Apply the first 3 m/s MC pusher-feedback envelope."""
    previous = np.asarray(previous, dtype=float)
    requested = np.asarray(requested, dtype=float).copy()
    requested[0] = np.clip(requested[0], 0.48, 0.56)
    requested[1] = np.clip(requested[1], 0.0, 0.10)
    # Gate A validates pusher-speed feedback close to level attitude. Live
    # ULogs showed that the wider generic MC envelope let small cross-track
    # errors drive an oscillatory roll response. Keep enough authority for
    # gentle attitude correction without letting lateral motion dominate the
    # 3 m/s forward-speed test.
    requested[2] = np.clip(0.25 * requested[2], -0.10, 0.10)
    requested[3:5] = np.clip(
        0.5 * requested[3:5],
        [-0.20, -0.15],
        [0.20, 0.15],
    )
    slew_per_second = np.array([0.10, 0.03, 0.20, 0.30, 0.20])
    return previous + np.clip(
        requested - previous,
        -slew_per_second * dt,
        slew_per_second * dt,
    )


def govern_pusher_forward_envelope(
    previous,
    limited,
    forward_speed: float,
    reference_speed: float,
    target_speed: float,
    pitch: float,
    dt: float = 0.05,
) -> np.ndarray:
    """Guard speed and near-level attitude around the reduced NMPC model.

    This is a robust safety layer around the reduced NMPC model. It remains
    The speed guard remains inactive inside a 0.10 m/s tracking band. The
    attitude guard starts above four degrees of forward pitch, well before the
    hard ten-degree watchdog used by Gate A.
    """
    previous = np.asarray(previous, dtype=float)
    result = np.asarray(limited, dtype=float).copy()
    pitch = float(pitch)
    overspeed = max(
        float(forward_speed) - float(reference_speed) - 0.10,
        float(forward_speed) - float(target_speed) - 0.05,
    )
    pitch_excess = max(pitch - np.deg2rad(4.0), 0.0)
    if overspeed <= 0.0 and pitch_excess <= 0.0:
        return result

    if overspeed > 0.0:
        # Reducing thrust is allowed faster than the conservative upward ramp;
        # the custom PX4 branch still applies its independent actuator slew.
        result[1] = max(0.0, previous[1] - 0.10 * float(dt))

    # Positive FLU pitch is nose-forward/down for the converted Gazebo state.
    # A negative q command moves that attitude back toward level. Preserve any
    # stronger braking request already produced by NMPC.
    if pitch_excess > 0.0:
        # Control-barrier behavior: once the aircraft reaches the soft
        # four-degree envelope, never pass a command that increases forward
        # pitch. Bypass the ordinary slew here because PX4's measured 75 ms
        # rate-loop delay otherwise lets a saturated command cross 10 degrees
        # before the outer loop can reverse it.
        barrier_rate = -float(np.clip(8.0 * pitch_excess, 0.0, 0.20))
        result[3] = min(result[3], barrier_rate)
    else:
        level_rate = -float(
            np.clip(0.8 * max(overspeed, 0.0), 0.0, 0.20)
        )
        desired_pitch_rate = min(result[3], level_rate)
        result[3] = previous[3] + np.clip(
            desired_pitch_rate - previous[3], -0.60 * dt, 0.60 * dt
        )
    return result


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
    # Gate A ULogs showed a slow 0.30 m upward drift during the brake phase
    # while the original 1 rad/s position loop still had ample thrust margin.
    # Double the natural-frequency-squared and retain critical damping so the
    # correction starts earlier without introducing a vertical oscillation.
    position_gain = 2.0
    velocity_gain = 2.0 * np.sqrt(position_gain)
    desired_acceleration = (
        -position_gain * float(altitude_error)
        - velocity_gain * float(vertical_speed)
    )
    return float(hover + desired_acceleration / acceleration_per_command)
