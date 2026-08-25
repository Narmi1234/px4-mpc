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
    estimated_offset_us: int | None = None
    max_translated_step_us: int = 250_000
    sync_anchor_px4_us: int = 0
    sync_anchor_wall_ns: int = 0
    previous_sync_px4_us: int = 0
    previous_sync_wall_ns: int = 0
    px4_rate_per_wall: float = 1.0

    @property
    def synchronized(self) -> bool:
        """Return whether a direct raw PX4 clock anchor is available."""
        return self.sync_anchor_px4_us > 0

    def update_timesync(
        self,
        translated_timestamp_us: int,
        offset_us: int,
        remote_timestamp_us: int | None = None,
        observed_offset_us: int | None = None,
        wall_ns: int | None = None,
    ) -> None:
        """Anchor the clock to a direct PX4 sample and estimate simulation rate."""
        self.estimated_offset_us = int(offset_us)
        if remote_timestamp_us is not None and observed_offset_us is not None:
            direct_px4_us = int(remote_timestamp_us) + int(observed_offset_us)
            if direct_px4_us > 0:
                # Unlike the filtered estimated offset, this sum is the raw
                # PX4 timestamp observed by the current timesync exchange.
                # It is allowed to correct a previously accepted forward jump.
                self.sync_anchor_px4_us = direct_px4_us
                if wall_ns is not None and int(wall_ns) > 0:
                    wall_ns = int(wall_ns)
                    if (
                        self.previous_sync_px4_us > 0
                        and self.previous_sync_wall_ns > 0
                        and direct_px4_us > self.previous_sync_px4_us
                        and wall_ns > self.previous_sync_wall_ns
                    ):
                        measured_rate = (
                            (direct_px4_us - self.previous_sync_px4_us)
                            * 1000.0
                            / (wall_ns - self.previous_sync_wall_ns)
                        )
                        if 0.05 <= measured_rate <= 3.0:
                            self.px4_rate_per_wall = measured_rate
                    self.previous_sync_px4_us = direct_px4_us
                    self.previous_sync_wall_ns = wall_ns
                    self.sync_anchor_wall_ns = wall_ns
                self.latest_px4_us = direct_px4_us
                return
        self.update_translated_timestamp(translated_timestamp_us)

    def advance_from_wall(self, wall_ns: int) -> int:
        """Interpolate PX4 boot time from the latest direct timesync anchor."""
        wall_ns = int(wall_ns)
        if self.sync_anchor_px4_us <= 0 or self.sync_anchor_wall_ns <= 0:
            return self.latest_px4_us
        wall_delta_ns = max(0, wall_ns - self.sync_anchor_wall_ns)
        self.latest_px4_us = self.sync_anchor_px4_us + int(
            wall_delta_ns * 1.0e-3 * self.px4_rate_per_wall
        )
        return self.latest_px4_us

    def translated_to_px4(self, timestamp_us: int) -> int:
        """Undo uXRCE-DDS timestamp synchronization to recover PX4 boot time."""
        if not self.synchronized:
            return 0
        return int(timestamp_us) + int(self.estimated_offset_us)

    def update_translated_timestamp(self, timestamp_us: int) -> int:
        """Observe a DDS timestamp and return accepted PX4 boot time or zero."""
        raw_timestamp_us = self.translated_to_px4(timestamp_us)
        if (
            raw_timestamp_us > 0
            and (
                self.latest_px4_us <= 0
                or raw_timestamp_us - self.latest_px4_us
                <= self.max_translated_step_us
            )
        ):
            self.update_px4_timestamp(raw_timestamp_us)
            return raw_timestamp_us
        return 0

    def update_px4_timestamp(self, timestamp_us: int) -> None:
        """Accept a positive, monotonic PX4 timestamp."""
        timestamp_us = int(timestamp_us)
        if timestamp_us > 0 and timestamp_us >= self.latest_px4_us:
            self.latest_px4_us = timestamp_us

    def start_offboard(self, observed_px4_us: int, wall_ns: int) -> None:
        """Start timing from a timestamp in the active PX4 message domain.

        Reject an event timestamp more than five seconds from the latest
        observed raw PX4 clock so mixed domains cannot create a huge false
        elapsed time.
        """
        observed_px4_us = int(observed_px4_us)
        if observed_px4_us <= 0 or (
            self.latest_px4_us > 0
            and abs(self.latest_px4_us - observed_px4_us) > 5_000_000
        ):
            observed_px4_us = self.latest_px4_us
        # A transient DDS-offset handover may have advanced latest_px4_us
        # before the matching TimesyncStatus arrived. The observed Offboard
        # transition is the authoritative zero for this interval.
        if observed_px4_us > 0:
            self.latest_px4_us = observed_px4_us
            self.sync_anchor_px4_us = observed_px4_us
            self.sync_anchor_wall_ns = int(wall_ns)
        self.offboard_start_px4_us = observed_px4_us
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
