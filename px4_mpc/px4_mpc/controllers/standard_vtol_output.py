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
    pusher_limit: float = 0.10,
) -> np.ndarray:
    """Apply the guarded MC pusher-feedback envelope."""
    previous = np.asarray(previous, dtype=float)
    requested = np.asarray(requested, dtype=float).copy()
    pusher_limit = float(pusher_limit)
    if not np.isfinite(pusher_limit) or pusher_limit <= 0.0:
        raise ValueError("pusher_limit must be positive and finite")
    requested[0] = np.clip(requested[0], 0.48, 0.56)
    requested[1] = np.clip(requested[1], 0.0, pusher_limit)
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


def limit_transition_command(
    previous,
    requested,
    dt: float = 0.05,
    pusher_limit: float = 0.30,
    pusher_slew: float = 0.05,
    apply_lift_blend: bool = False,
) -> np.ndarray:
    """Bound Gate D commands without Gate A's near-level pitch barrier."""
    previous = np.asarray(previous, dtype=float)
    requested = np.asarray(requested, dtype=float).copy()
    collective_floor = 0.0 if apply_lift_blend else 0.48
    requested[0] = np.clip(requested[0], collective_floor, 0.56)
    requested[1] = np.clip(requested[1], 0.0, pusher_limit)
    requested[2:5] = np.clip(
        requested[2:5], [-0.12, -0.25, -0.15], [0.12, 0.25, 0.15]
    )
    collective_slew = 0.30 if apply_lift_blend else 0.10
    slew_per_second = np.array(
        [collective_slew, float(pusher_slew), 0.20, 0.35, 0.20]
    )
    return previous + np.clip(
        requested - previous,
        -slew_per_second * dt,
        slew_per_second * dt,
    )


def govern_transition_speed(
    previous,
    limited,
    forward_speed: float,
    reference_speed: float,
    dt: float = 0.05,
) -> np.ndarray:
    """Remove pusher promptly when Gate D exceeds its speed reference."""
    previous = np.asarray(previous, dtype=float)
    result = np.asarray(limited, dtype=float).copy()
    # Attempt 06 proved that a 0.75 m/s deadband and 0.10/s reduction were
    # slower than the live Standard VTOL acceleration: the aircraft entered
    # FW at 13.27 m/s and immediately crossed the 14 m/s watchdog.  Match the
    # reduction to the independently enforced PX4 pusher slew once the
    # measured speed is 0.25 m/s above reference.
    if float(forward_speed) > float(reference_speed) + 0.25:
        result[1] = max(0.0, previous[1] - 0.33 * float(dt))
    return result


def govern_transition_pitch(
    previous,
    limited,
    pitch: float,
    reference_pitch: float,
    dt: float = 0.05,
    rate_limit: float = 0.10,
    rate_slew: float = 0.20,
    position_gain: float = 1.0,
) -> np.ndarray:
    """Close a slow pitch loop around the rate-input transition model.

    The 10-state plant treats body rate as the inner-loop output. The live PX4
    rate loop and aerodynamic surfaces add a measured delay, so an unbounded
    NMPC rate reversal produced the Gate D attempt-02 pitch oscillation. This
    The caller selects phase-specific rate and slew bounds. Front transition
    keeps the validated 0.10 rad/s and 0.20 rad/s^2 envelope; confirmed FW and
    back transition use the wider stock-PX4-informed envelope needed after
    lift-motor shutdown. Inside a four-degree corridor NMPC retains authority;
    outside it, only rates back into the corridor are accepted.
    """
    previous = np.asarray(previous, dtype=float)
    result = np.asarray(limited, dtype=float).copy()
    pitch = float(pitch)
    reference_pitch = float(reference_pitch)
    rate_limit = float(rate_limit)
    rate_slew = float(rate_slew)
    position_gain = float(position_gain)
    if rate_limit <= 0.0 or rate_slew <= 0.0 or position_gain <= 0.0:
        raise ValueError("pitch governor limits and gain must be positive")
    soft_error = np.deg2rad(4.0)
    lower_rate = float(
        np.clip(
            position_gain * (reference_pitch - soft_error - pitch),
            -rate_limit,
            rate_limit,
        )
    )
    upper_rate = float(
        np.clip(
            position_gain * (reference_pitch + soft_error - pitch),
            -rate_limit,
            rate_limit,
        )
    )
    target_rate = float(
        np.clip(
            np.clip(result[3], -rate_limit, rate_limit),
            lower_rate,
            upper_rate,
        )
    )
    result[3] = previous[3] + np.clip(
        target_rate - previous[3],
        -rate_slew * float(dt),
        rate_slew * float(dt),
    )
    return result


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
    if overspeed > 0.0:
        # Reducing thrust is allowed faster than the conservative upward ramp;
        # the custom PX4 branch still applies its independent actuator slew.
        result[1] = max(0.0, previous[1] - 0.10 * float(dt))

    # Continuous symmetric control barrier. Positive FLU pitch is nose-down;
    # negative FLU pitch is nose-up. The earlier one-sided barrier protected
    # only the first case and allowed the braking/settle maneuver to reach the
    # opposite ten-degree watchdog. Clip q between two smooth bounds so either
    # side begins leveling at four degrees. Pusher reduction alone handles
    # overspeed; it must not inject a pitch command with the wrong sign.
    soft_pitch = np.deg2rad(4.0)
    lower_rate = float(
        np.clip(4.0 * (-soft_pitch - pitch), -0.20, 0.20)
    )
    upper_rate = float(
        np.clip(4.0 * (soft_pitch - pitch), -0.20, 0.20)
    )
    result[3] = np.clip(result[3], lower_rate, upper_rate)
    return result


def govern_pusher_forward_lateral(
    previous,
    limited,
    cross_track_error: float,
    cross_track_speed: float,
    roll: float,
    dt: float = 0.05,
) -> np.ndarray:
    """Apply a damped Gate A cross-track loop through the roll-rate command.

    The reduced NMPC's lateral channel proved too sensitive to the measured
    PX4 rate delay during braking.  Keep NMPC responsible for forward speed,
    but close this deliberately slow outer loop around signed cross-track
    position, velocity and measured roll.
    """
    previous = np.asarray(previous, dtype=float)
    result = np.asarray(limited, dtype=float).copy()
    desired_lateral_acceleration = float(
        np.clip(
            -0.35 * float(cross_track_error)
            - 1.10 * float(cross_track_speed),
            -0.60,
            0.60,
        )
    )
    # In the ENU/FLU model, positive roll accelerates toward negative body-y.
    desired_roll = float(
        np.clip(
            -desired_lateral_acceleration / 9.81,
            -np.deg2rad(4.0),
            np.deg2rad(4.0),
        )
    )
    target_roll_rate = float(
        np.clip(1.5 * (desired_roll - float(roll)), -0.08, 0.08)
    )
    result[2] = previous[2] + np.clip(
        target_roll_rate - previous[2], -0.20 * dt, 0.20 * dt
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


def pretransition_lift_unloading(
    forward_speed: float,
    maximum_unloading: float = 0.020,
) -> float:
    """Return the B1-ULog-bounded MC lift-unloading feedforward.

    The schedule is zero through 3 m/s, reaches the measured 0.010 command
    reduction at 5 m/s, and is conservatively capped at 0.020 by 8 m/s. Cubic
    smoothstep segments avoid a collective derivative discontinuity.
    """
    speed = max(0.0, float(forward_speed))
    maximum_unloading = float(maximum_unloading)
    if not np.isfinite(maximum_unloading) or not 0.0 <= maximum_unloading <= 0.03:
        raise ValueError("maximum_unloading must be finite and within [0, 0.03]")

    def smoothstep(value: float) -> float:
        value = float(np.clip(value, 0.0, 1.0))
        return value * value * (3.0 - 2.0 * value)

    measured_at_five = min(0.010, maximum_unloading)
    if speed <= 3.0:
        return 0.0
    if speed < 5.0:
        return measured_at_five * smoothstep((speed - 3.0) / 2.0)
    if speed < 8.0:
        return measured_at_five + (
            maximum_unloading - measured_at_five
        ) * smoothstep((speed - 5.0) / 3.0)
    return maximum_unloading


def pretransition_lift_command(
    plant,
    altitude_error: float,
    vertical_speed: float,
    forward_speed: float,
    maximum_unloading: float = 0.020,
) -> tuple[float, float]:
    """Combine proven altitude feedback with bounded B2 lift unloading."""
    unloading = pretransition_lift_unloading(
        forward_speed, maximum_unloading=maximum_unloading
    )
    command = vertical_hover_lift(plant, altitude_error, vertical_speed) - unloading
    return float(np.clip(command, 0.48, 0.56)), unloading
