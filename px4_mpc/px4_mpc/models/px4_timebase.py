"""PX4 timestamp timebase for SITL profiles and wall-time watchdog diagnostics."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Px4Timebase:
    """Track one Offboard interval in PX4 and local wall time."""

    latest_px4_us: int = 0
    offboard_start_px4_us: int = 0
    offboard_start_wall_ns: int = 0
    last_px4_elapsed: float = 0.0
    last_wall_elapsed: float = 0.0

    def update_px4_timestamp(self, timestamp_us: int) -> None:
        """Accept a positive, monotonic PX4 timestamp."""
        timestamp_us = int(timestamp_us)
        if timestamp_us > 0 and timestamp_us >= self.latest_px4_us:
            self.latest_px4_us = timestamp_us

    def start_offboard(self, transition_px4_us: int, wall_ns: int) -> None:
        """Start timing from PX4's exact nav-state transition timestamp."""
        transition_px4_us = int(transition_px4_us)
        if transition_px4_us <= 0:
            transition_px4_us = self.latest_px4_us
        self.offboard_start_px4_us = transition_px4_us
        self.offboard_start_wall_ns = int(wall_ns)
        self.last_px4_elapsed = 0.0
        self.last_wall_elapsed = 0.0

    @property
    def active(self) -> bool:
        """Return whether an Offboard interval is currently timed."""
        return self.offboard_start_px4_us > 0

    def px4_elapsed(self) -> float:
        """Return elapsed plant/simulation time in seconds."""
        if not self.active:
            return self.last_px4_elapsed
        return max(
            0.0,
            (self.latest_px4_us - self.offboard_start_px4_us) * 1.0e-6,
        )

    def wall_elapsed(self, wall_ns: int) -> float:
        """Return elapsed local wall time in seconds."""
        if not self.active or self.offboard_start_wall_ns <= 0:
            return self.last_wall_elapsed
        return max(0.0, (int(wall_ns) - self.offboard_start_wall_ns) * 1.0e-9)

    def realtime_factor(self, wall_ns: int) -> float:
        """Return PX4 elapsed time divided by wall elapsed time."""
        wall_elapsed = self.wall_elapsed(wall_ns)
        if wall_elapsed <= 1.0e-9:
            return float("nan")
        return self.px4_elapsed() / wall_elapsed

    def stop_offboard(self, wall_ns: int) -> tuple[float, float]:
        """Store and return final PX4 and wall durations."""
        self.last_px4_elapsed = self.px4_elapsed()
        self.last_wall_elapsed = self.wall_elapsed(wall_ns)
        self.offboard_start_px4_us = 0
        self.offboard_start_wall_ns = 0
        return self.last_px4_elapsed, self.last_wall_elapsed

    def reset(self) -> None:
        """Clear the active interval while retaining the latest PX4 clock."""
        self.offboard_start_px4_us = 0
        self.offboard_start_wall_ns = 0
        self.last_px4_elapsed = 0.0
        self.last_wall_elapsed = 0.0
