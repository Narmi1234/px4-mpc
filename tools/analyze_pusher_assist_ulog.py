#!/usr/bin/env python3
"""Confirm that PX4 actually used the pusher during the latest Offboard gate."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from pyulog import ULog


OFFBOARD_NAV_STATE = 14
MC_VTOL_STATE = 3


def dataset(ulog: ULog, name: str):
    matches = [item for item in ulog.data_list if item.name == name]
    if not matches:
        raise RuntimeError(f"ULog is missing required dataset: {name}")
    return matches[0].data


def last_offboard_interval(status) -> tuple[int, int]:
    timestamps = np.asarray(status["timestamp"], dtype=np.int64)
    states = np.asarray(status["nav_state"], dtype=int)
    indices = np.flatnonzero(states == OFFBOARD_NAV_STATE)
    if not len(indices):
        raise RuntimeError("ULog contains no Offboard interval")
    end_of_run = int(indices[-1])
    start_index = end_of_run
    while start_index > 0 and states[start_index - 1] == OFFBOARD_NAV_STATE:
        start_index -= 1
    later = np.flatnonzero(
        (np.arange(len(states)) > end_of_run) & (states != OFFBOARD_NAV_STATE)
    )
    if not len(later):
        raise RuntimeError("latest Offboard interval has no recorded exit")
    return int(timestamps[start_index]), int(timestamps[int(later[0])])


def window(data, start: int, end: int) -> np.ndarray:
    timestamps = np.asarray(data["timestamp"], dtype=np.int64)
    return (timestamps >= start) & (timestamps <= end)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("ulog", type=Path)
    args = parser.parse_args()
    ulog = ULog(
        str(args.ulog),
        message_name_filter_list=[
            "actuator_motors",
            "vehicle_local_position",
            "vehicle_status",
            "vtol_vehicle_status",
        ],
    )
    status = dataset(ulog, "vehicle_status")
    motors = dataset(ulog, "actuator_motors")
    local_position = dataset(ulog, "vehicle_local_position")
    vtol = dataset(ulog, "vtol_vehicle_status")
    start, end = last_offboard_interval(status)
    motor_window = window(motors, start, end)
    position_window = window(local_position, start, end)
    if not np.any(motor_window) or not np.any(position_window):
        raise RuntimeError("Offboard interval has no actuator or position samples")

    pusher = np.asarray(motors["control[4]"], dtype=float)[motor_window]
    vx = np.asarray(local_position["vx"], dtype=float)[position_window]
    vy = np.asarray(local_position["vy"], dtype=float)[position_window]
    z = np.asarray(local_position["z"], dtype=float)[position_window]
    vtol_times = np.asarray(vtol["timestamp"], dtype=np.int64)
    state_before = np.flatnonzero(vtol_times <= start)
    state_changes = np.flatnonzero((vtol_times >= start) & (vtol_times <= end))
    relevant = np.unique(
        np.r_[state_before[-1:] if len(state_before) else [], state_changes]
    ).astype(int)
    vtol_states = np.asarray(vtol["vehicle_vtol_state"], dtype=int)[relevant]

    duration = (end - start) * 1e-6
    finite_pusher = pusher[np.isfinite(pusher)]
    peak_pusher = float(np.max(finite_pusher)) if len(finite_pusher) else float("nan")
    peak_speed = float(np.nanmax(np.hypot(vx, vy)))
    altitude_span = float(np.nanmax(z) - np.nanmin(z))
    checks = {
        "duration": duration >= 12.5,
        "pusher_engaged": 0.005 <= peak_pusher <= 0.15,
        "speed_reached": peak_speed >= 2.5,
        "altitude_span": altitude_span <= 0.6,
        "mc_only": len(vtol_states) > 0 and np.all(vtol_states == MC_VTOL_STATE),
    }
    print(f"offboard_duration={duration:.3f}s")
    print(f"peak_pusher_actuator={peak_pusher:.4f}")
    print(f"peak_horizontal_speed={peak_speed:.3f}m/s")
    print(f"altitude_span={altitude_span:.3f}m")
    print(f"vtol_states={vtol_states.tolist()}")
    for name, passed in checks.items():
        print(f"{name}={'PASS' if passed else 'FAIL'}")
    if not all(checks.values()):
        raise SystemExit("ulog_gate=FAIL")
    print("ulog_gate=PASS")


if __name__ == "__main__":
    main()
