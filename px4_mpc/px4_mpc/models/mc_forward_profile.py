"""Bounded multicopter forward-speed profile used before VTOL transition."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class McForwardSample:
    """One point on the one-dimensional forward test profile."""

    distance: float
    speed: float
    acceleration: float
    phase: str


@dataclass(frozen=True)
class McForwardProfile:
    """Accelerate, briefly hold speed, then stop at a new hover point."""

    target_speed: float = 2.0
    acceleration: float = 1.0
    hold_seconds: float = 1.0
    start_delay_seconds: float = 2.0

    def __post_init__(self) -> None:
        if not np.isfinite(self.target_speed) or self.target_speed <= 0.0:
            raise ValueError("target_speed must be positive and finite")
        if not np.isfinite(self.acceleration) or self.acceleration <= 0.0:
            raise ValueError("acceleration must be positive and finite")
        if not np.isfinite(self.hold_seconds) or self.hold_seconds < 0.0:
            raise ValueError("hold_seconds must be nonnegative and finite")
        if not np.isfinite(self.start_delay_seconds) or self.start_delay_seconds < 0.0:
            raise ValueError("start_delay_seconds must be nonnegative and finite")

    @property
    def acceleration_seconds(self) -> float:
        # A half-cosine velocity ramp has peak acceleration
        # V*pi/(2*T). Choose T so the configured acceleration is never exceeded.
        return np.pi * self.target_speed / (2.0 * self.acceleration)

    @property
    def motion_seconds(self) -> float:
        return 2.0 * self.acceleration_seconds + self.hold_seconds

    @property
    def profile_seconds(self) -> float:
        return self.start_delay_seconds + self.motion_seconds

    @property
    def final_distance(self) -> float:
        return self.target_speed * (
            self.acceleration_seconds + self.hold_seconds
        )

    def sample(self, time_seconds: float) -> McForwardSample:
        """Evaluate the continuous position/velocity profile at ``time``."""
        time_seconds = max(0.0, float(time_seconds))
        if time_seconds < self.start_delay_seconds:
            return McForwardSample(
                distance=0.0,
                speed=0.0,
                acceleration=0.0,
                phase="initial_hover",
            )
        time_seconds -= self.start_delay_seconds
        ramp = self.acceleration_seconds
        omega = np.pi / ramp
        ramp_distance = 0.5 * self.target_speed * ramp
        if time_seconds < ramp:
            return McForwardSample(
                distance=0.5
                * self.target_speed
                * (time_seconds - np.sin(omega * time_seconds) / omega),
                speed=0.5
                * self.target_speed
                * (1.0 - np.cos(omega * time_seconds)),
                acceleration=0.5
                * self.target_speed
                * omega
                * np.sin(omega * time_seconds),
                phase="accelerate",
            )
        if time_seconds < ramp + self.hold_seconds:
            hold_time = time_seconds - ramp
            return McForwardSample(
                distance=ramp_distance + self.target_speed * hold_time,
                speed=self.target_speed,
                acceleration=0.0,
                phase="hold_speed",
            )
        if time_seconds < self.motion_seconds:
            brake_time = time_seconds - ramp - self.hold_seconds
            return McForwardSample(
                distance=(
                    ramp_distance
                    + self.target_speed * self.hold_seconds
                    + 0.5
                    * self.target_speed
                    * (brake_time + np.sin(omega * brake_time) / omega)
                ),
                speed=0.5
                * self.target_speed
                * (1.0 + np.cos(omega * brake_time)),
                acceleration=-0.5
                * self.target_speed
                * omega
                * np.sin(omega * brake_time),
                phase="brake",
            )
        return McForwardSample(
            distance=self.final_distance,
            speed=0.0,
            acceleration=0.0,
            phase="settle_hover",
        )


def mc_forward_reference_state(
    hold_state: np.ndarray,
    forward_direction: np.ndarray,
    sample: McForwardSample,
    gravity: float,
) -> np.ndarray:
    """Map a scalar profile sample into the 10-state ENU/FLU reference."""
    hold_state = np.asarray(hold_state, dtype=float)
    direction = np.asarray(forward_direction, dtype=float)
    if hold_state.shape != (10,):
        raise ValueError("hold_state must have shape (10,)")
    if direction.shape != (2,) or not np.isclose(np.linalg.norm(direction), 1.0):
        raise ValueError("forward_direction must be a unit 2-vector")
    reference = hold_state.copy()
    reference[0:2] += direction * sample.distance
    reference[3:5] = direction * sample.speed
    qw, qx, qy, qz = hold_state[6:10]
    yaw = np.arctan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    pitch = np.arctan2(sample.acceleration, gravity)
    cy, sy = np.cos(0.5 * yaw), np.sin(0.5 * yaw)
    cp, sp = np.cos(0.5 * pitch), np.sin(0.5 * pitch)
    reference[6:10] = [cy * cp, -sy * sp, cy * sp, sy * cp]
    return reference


def pusher_forward_reference_state(
    hold_state: np.ndarray,
    forward_direction: np.ndarray,
    sample: McForwardSample,
) -> np.ndarray:
    """Map a moving profile to a level-attitude pusher test reference."""
    hold_state = np.asarray(hold_state, dtype=float)
    direction = np.asarray(forward_direction, dtype=float)
    if hold_state.shape != (10,):
        raise ValueError("hold_state must have shape (10,)")
    if direction.shape != (2,) or not np.isclose(np.linalg.norm(direction), 1.0):
        raise ValueError("forward_direction must be a unit 2-vector")
    reference = hold_state.copy()
    reference[0:2] += direction * sample.distance
    reference[3:5] = direction * sample.speed
    # hold_state is levelled when the gate is captured. Keeping this attitude
    # reference makes the optimizer use pusher thrust instead of reproducing
    # the earlier tilt-only MC acceleration test.
    reference[6:10] = hold_state[6:10]
    return reference


def pusher_forward_feedforward(
    plant,
    speed: float,
    acceleration: float,
    command_limit: float = 0.10,
) -> float:
    """Return bounded level-flight pusher feedforward for one profile sample."""
    speed = max(0.0, float(speed))
    acceleration = float(acceleration)
    pusher = plant.motors[4]
    trim = plant.nominal_level_flight_trim(speed)
    trim_speed = pusher.target_speed(trim.pusher_command)
    trim_thrust = pusher.motor_constant * trim_speed**2
    required_thrust = max(0.0, trim_thrust + plant.mass * acceleration)
    required_speed = np.sqrt(required_thrust / pusher.motor_constant)
    return float(
        np.clip(pusher.normalized_command(required_speed), 0.0, command_limit)
    )
