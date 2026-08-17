"""Small bounded pusher pulse used to validate the custom PX4 interface."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ExternalPusherSample:
    command: float
    phase: str


@dataclass(frozen=True)
class ExternalPusherProfile:
    peak_command: float = 0.05
    slew_per_second: float = 0.02
    hold_seconds: float = 2.0
    start_delay_seconds: float = 2.0

    def __post_init__(self) -> None:
        values = (
            self.peak_command,
            self.slew_per_second,
            self.hold_seconds,
            self.start_delay_seconds,
        )
        if not all(np.isfinite(values)):
            raise ValueError("profile values must be finite")
        if not 0.0 < self.peak_command <= 0.05:
            raise ValueError("peak_command must be in (0, 0.05]")
        if self.slew_per_second <= 0.0:
            raise ValueError("slew_per_second must be positive")
        if self.hold_seconds < 0.0 or self.start_delay_seconds < 0.0:
            raise ValueError("profile times must be nonnegative")

    @property
    def ramp_seconds(self) -> float:
        return self.peak_command / self.slew_per_second

    @property
    def profile_seconds(self) -> float:
        return self.start_delay_seconds + 2.0 * self.ramp_seconds + self.hold_seconds

    def sample(self, time_seconds: float) -> ExternalPusherSample:
        time_seconds = max(0.0, float(time_seconds))
        if time_seconds < self.start_delay_seconds:
            return ExternalPusherSample(0.0, "initial_hover")
        time_seconds -= self.start_delay_seconds
        if time_seconds < self.ramp_seconds:
            return ExternalPusherSample(
                self.slew_per_second * time_seconds, "pusher_ramp_up"
            )
        time_seconds -= self.ramp_seconds
        if time_seconds < self.hold_seconds:
            return ExternalPusherSample(self.peak_command, "pusher_hold")
        time_seconds -= self.hold_seconds
        if time_seconds < self.ramp_seconds:
            return ExternalPusherSample(
                self.peak_command - self.slew_per_second * time_seconds,
                "pusher_ramp_down",
            )
        return ExternalPusherSample(0.0, "settle_hover")
