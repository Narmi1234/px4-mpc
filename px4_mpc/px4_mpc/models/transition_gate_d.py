"""Pure Gate D state machine and reference schedules."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from px4_mpc.models.mc_forward_profile import McForwardSample


VTOL_TRANSITION_TO_FW = 1
VTOL_TRANSITION_TO_MC = 2
VTOL_MC = 3
VTOL_FW = 4


@dataclass(frozen=True)
class GateDUpdate:
    """One state-machine update and optional PX4 transition request."""

    state: str
    transition_request: int | None = None
    completed: bool = False
    failed: bool = False


def _half_cosine(
    start_speed: float,
    end_speed: float,
    elapsed: float,
    peak_acceleration: float,
) -> tuple[float, float, float, float]:
    """Return distance, speed, acceleration and ramp duration."""
    delta = end_speed - start_speed
    if abs(delta) <= 1.0e-12:
        return start_speed * max(0.0, elapsed), start_speed, 0.0, 0.0
    duration = math.pi * abs(delta) / (2.0 * peak_acceleration)
    time = float(np.clip(elapsed, 0.0, duration))
    omega = math.pi / duration
    blend = 0.5 * (1.0 - math.cos(omega * time))
    speed = start_speed + delta * blend
    acceleration = 0.5 * delta * omega * math.sin(omega * time)
    distance = (
        start_speed * time
        + 0.5 * delta * (time - math.sin(omega * time) / omega)
    )
    if elapsed > duration:
        distance += end_speed * (elapsed - duration)
        acceleration = 0.0
    return distance, speed, acceleration, duration


class GateDStateMachine:
    """Guarded first front/back transition sequence independent of ROS."""

    maximum_seconds = 90.0
    front_transition_timeout = 12.0
    fw_hold_seconds = 5.0
    condition_hold_seconds = 1.0
    recovery_stop_seconds = 2.0

    def __init__(self) -> None:
        self.state = "idle"
        self.started_s = 0.0
        self.entered_s = 0.0
        self.front_started_s = 0.0
        self.brake_started_s = 0.0
        self.front_start_distance = 0.0
        self.front_start_speed = 8.0
        self.brake_start_distance = 0.0
        self.brake_start_speed = 12.0
        self.condition_since_s: float | None = None
        self.abort_reason = "none"

    def start(self, now_s: float) -> None:
        self.state = "mc_accelerate"
        self.started_s = float(now_s)
        self.entered_s = float(now_s)
        self.front_started_s = 0.0
        self.brake_started_s = 0.0
        self.front_start_distance = 0.0
        self.front_start_speed = 8.0
        self.brake_start_distance = 0.0
        self.brake_start_speed = 12.0
        self.condition_since_s = None
        self.abort_reason = "none"

    def _enter(self, state: str, now_s: float) -> None:
        self.state = state
        self.entered_s = float(now_s)
        self.condition_since_s = None

    def abort(self, reason: str, now_s: float, vtol_state: int) -> GateDUpdate:
        self.abort_reason = reason
        self._enter("abort_recovery", now_s)
        request = VTOL_MC if vtol_state != VTOL_MC else None
        return GateDUpdate(self.state, transition_request=request)

    def update(
        self,
        now_s: float,
        vtol_state: int,
        forward_speed: float,
        airspeed: float,
        commanded_pusher: float = 0.0,
    ) -> GateDUpdate:
        now_s = float(now_s)
        if self.state == "idle":
            return GateDUpdate(self.state)
        if self.state == "abort_recovery":
            if vtol_state == VTOL_MC:
                self._enter("failed", now_s)
                return GateDUpdate(self.state, failed=True)
            return GateDUpdate(self.state)
        if now_s - self.started_s > self.maximum_seconds:
            return self.abort("gate_d_timeout", now_s, vtol_state)

        if self.state == "mc_accelerate":
            if vtol_state != VTOL_MC:
                return self.abort("unexpected_vtol_state_before_front", now_s, vtol_state)
            ready = forward_speed >= 7.5 and airspeed >= 7.5
            if ready and self.condition_since_s is None:
                self.condition_since_s = now_s
            elif not ready:
                self.condition_since_s = None
            if (
                self.condition_since_s is not None
                and now_s - self.condition_since_s >= self.condition_hold_seconds
            ):
                current = self.sample(now_s)
                self.front_start_distance = current.distance
                self.front_start_speed = current.speed
                self._enter("front_transition", now_s)
                self.front_started_s = now_s
                return GateDUpdate(self.state, transition_request=VTOL_FW)

        elif self.state == "front_transition":
            if vtol_state == VTOL_FW:
                self._enter("fw_hold", now_s)
            elif vtol_state not in (VTOL_MC, VTOL_TRANSITION_TO_FW):
                return self.abort("invalid_front_transition_state", now_s, vtol_state)
            elif now_s - self.entered_s > self.front_transition_timeout:
                return self.abort("front_transition_timeout", now_s, vtol_state)

        elif self.state == "fw_hold":
            if vtol_state != VTOL_FW:
                return self.abort("left_fw_before_back_request", now_s, vtol_state)
            if now_s - self.entered_s >= self.fw_hold_seconds:
                current = self.sample(now_s)
                self.brake_start_distance = current.distance
                self.brake_start_speed = current.speed
                self._enter("back_transition", now_s)
                self.brake_started_s = now_s
                return GateDUpdate(self.state, transition_request=VTOL_MC)

        elif self.state == "back_transition":
            if vtol_state == VTOL_MC:
                self._enter("mc_recovered", now_s)
            elif vtol_state not in (VTOL_FW, VTOL_TRANSITION_TO_MC):
                return self.abort("invalid_back_transition_state", now_s, vtol_state)

        elif self.state == "mc_recovered":
            if vtol_state != VTOL_MC:
                return self.abort("left_mc_after_recovery", now_s, vtol_state)
            stopped = (
                abs(forward_speed) <= 0.5
                and abs(commanded_pusher) <= 0.005
            )
            if stopped and self.condition_since_s is None:
                self.condition_since_s = now_s
            elif not stopped:
                self.condition_since_s = None
            if (
                self.condition_since_s is not None
                and now_s - self.condition_since_s >= self.recovery_stop_seconds
            ):
                self._enter("complete", now_s)
                return GateDUpdate(self.state, completed=True)

        return GateDUpdate(self.state)

    def sample(self, now_s: float) -> McForwardSample:
        """Return the speed-focused reference for the current state."""
        if self.state == "mc_accelerate":
            elapsed = max(0.0, now_s - self.started_s)
            distance, speed, acceleration, _ = _half_cosine(0.0, 8.0, elapsed, 0.50)
        elif self.state in ("front_transition", "fw_hold"):
            elapsed = max(0.0, now_s - self.front_started_s)
            distance, speed, acceleration, _ = _half_cosine(
                self.front_start_speed, 12.0, elapsed, 0.50
            )
            distance += self.front_start_distance
        elif self.state in ("back_transition", "mc_recovered"):
            elapsed = max(0.0, now_s - self.brake_started_s)
            distance, speed, acceleration, _ = _half_cosine(
                self.brake_start_speed, 0.0, elapsed, 0.75
            )
            distance += self.brake_start_distance
        else:
            distance, speed, acceleration = 0.0, 0.0, 0.0
        return McForwardSample(distance, speed, acceleration, self.state)


def px4_mc_weight(vtol_state: int, airspeed: float, phase_seconds: float) -> float:
    """Approximate Standard VTOL's measured/default lift-motor blend."""
    if vtol_state == VTOL_MC:
        return 1.0
    if vtol_state == VTOL_FW:
        return 0.0
    if vtol_state == VTOL_TRANSITION_TO_FW:
        return float(np.clip((10.0 - airspeed) / 2.0, 0.0, 1.0))
    if vtol_state == VTOL_TRANSITION_TO_MC:
        return float(np.clip(phase_seconds / 3.0, 0.0, 1.0))
    return 1.0


def transition_pitch_and_elevator(
    airspeed: float, lift_weight: float, vtol_state: int | None = None
) -> tuple[float, float]:
    """Return phase-correct pitch and elevator schedules.

    The identified trim corridor belongs to established wing-borne flight.
    Stock-PX4 ULogs show that the Standard VTOL stays essentially level during
    the short front transition. Applying the FW trim before FW confirmation
    excites the live inner rate loop just as lift-rotor authority is removed.
    """
    if vtol_state == VTOL_TRANSITION_TO_FW:
        return 0.0, 0.0
    speeds = np.array([0.0, 8.0, 9.0, 10.0, 11.0, 12.0, 15.0])
    pitch_deg = np.array([0.0, -8.25, -8.0, -7.319, -5.463, -4.049, -1.365])
    elevator_deg = np.array([0.0, 44.25, 43.88, 41.63, 32.06, 25.14, 12.91])
    speed = float(np.clip(airspeed, speeds[0], speeds[-1]))
    fw_weight = 1.0 - float(np.clip(lift_weight, 0.0, 1.0))
    return (
        math.radians(float(np.interp(speed, speeds, pitch_deg))) * fw_weight,
        math.radians(float(np.interp(speed, speeds, elevator_deg))) * fw_weight,
    )


def transition_pusher_trim(airspeed: float, lift_weight: float) -> float:
    """Return the identified corridor pusher, faded in with wing-borne weight.

    The MC endpoint is left at zero here because the existing dynamic
    feedforward remains valid before transition. The caller blends this trim
    with that MC feedforward using the same PX4 lift weight.
    """
    speeds = np.array([0.0, 5.0, 8.0, 9.0, 10.0, 12.0, 15.0, 18.0, 22.0])
    pusher = np.array(
        [0.0, 0.2902, 0.2880, 0.2825, 0.2657, 0.2660, 0.2675, 0.2696, 0.2728]
    )
    speed = float(np.clip(airspeed, speeds[0], speeds[-1]))
    fw_weight = 1.0 - float(np.clip(lift_weight, 0.0, 1.0))
    return fw_weight * float(np.interp(speed, speeds, pusher))
