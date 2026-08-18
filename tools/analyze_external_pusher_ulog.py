#!/usr/bin/env python3
"""Verify motor 5 followed the first custom external-pusher pulse in ULog."""

import argparse
from pathlib import Path

import numpy as np
from pyulog import ULog


OFFBOARD_NAV_STATE = 14
MC_VTOL_STATE = 3
# The ROS node currently schedules the 12 s gate in wall time, while PX4 ULog
# timestamps advance in Gazebo simulation time.  On a loaded machine Gazebo can
# run below real time; 10 s still leaves enough simulated time for the complete
# bounded pulse and its zero-command settle window.  The actuator checks below
# independently prove that the pulse reached its peak and returned to zero.
MIN_SITL_OFFBOARD_DURATION = 10.0


def get_data(ulog: ULog, name: str):
    matches = [item.data for item in ulog.data_list if item.name == name]
    if not matches:
        raise RuntimeError(f"ULog is missing {name}")
    return matches[0]


def last_offboard_interval(status) -> tuple[int, int]:
    timestamps = np.asarray(status["timestamp"], dtype=np.int64)
    states = np.asarray(status["nav_state"], dtype=int)
    indices = np.flatnonzero(states == OFFBOARD_NAV_STATE)
    if not len(indices):
        raise RuntimeError("ULog contains no Offboard interval")
    end_index = int(indices[-1])
    start_index = end_index
    while start_index > 0 and states[start_index - 1] == OFFBOARD_NAV_STATE:
        start_index -= 1
    exits = np.flatnonzero(
        (np.arange(len(states)) > end_index) & (states != OFFBOARD_NAV_STATE)
    )
    if not len(exits):
        raise RuntimeError("latest Offboard interval has no recorded exit")
    return int(timestamps[start_index]), int(timestamps[int(exits[0])])


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
    status = get_data(ulog, "vehicle_status")
    motors = get_data(ulog, "actuator_motors")
    position = get_data(ulog, "vehicle_local_position")
    vtol = get_data(ulog, "vtol_vehicle_status")
    start, end = last_offboard_interval(status)

    motor_time = np.asarray(motors["timestamp"], dtype=np.int64)
    motor_mask = (motor_time >= start) & (motor_time <= end)
    pusher = np.asarray(motors["control[4]"], dtype=float)[motor_mask]
    finite_pusher = pusher[np.isfinite(pusher)]
    final_window = np.asarray(motors["control[4]"], dtype=float)[
        (motor_time >= end - 1_000_000) & (motor_time <= end)
    ]
    finite_final = final_window[np.isfinite(final_window)]

    position_time = np.asarray(position["timestamp"], dtype=np.int64)
    position_mask = (position_time >= start) & (position_time <= end)
    vx = np.asarray(position["vx"], dtype=float)[position_mask]
    vy = np.asarray(position["vy"], dtype=float)[position_mask]
    z = np.asarray(position["z"], dtype=float)[position_mask]

    vtol_time = np.asarray(vtol["timestamp"], dtype=np.int64)
    before = np.flatnonzero(vtol_time <= start)
    during = np.flatnonzero((vtol_time >= start) & (vtol_time <= end))
    relevant = np.unique(np.r_[before[-1:] if len(before) else [], during]).astype(int)
    states = np.asarray(vtol["vehicle_vtol_state"], dtype=int)[relevant]

    if not len(vx):
        raise RuntimeError("Offboard interval has no position samples")
    duration = (end - start) * 1e-6
    peak_pusher = float(np.max(finite_pusher)) if len(finite_pusher) else float("nan")
    final_pusher = float(np.max(np.abs(finite_final))) if len(finite_final) else 0.0
    peak_speed = float(np.max(np.hypot(vx, vy)))
    altitude_span = float(np.max(z) - np.min(z))
    checks = {
        "duration": duration >= MIN_SITL_OFFBOARD_DURATION,
        "pusher_peak": 0.04 <= peak_pusher <= 0.055,
        "pusher_returned_zero": abs(final_pusher) <= 0.005,
        "speed_limit": peak_speed <= 1.5,
        "altitude_span": altitude_span <= 0.6,
        "mc_only": len(states) > 0 and np.all(states == MC_VTOL_STATE),
    }
    print(f"offboard_duration={duration:.3f}s")
    print(f"peak_pusher_actuator={peak_pusher:.4f}")
    print(f"final_pusher_actuator={final_pusher:.4f}")
    print(f"peak_horizontal_speed={peak_speed:.3f}m/s")
    print(f"altitude_span={altitude_span:.3f}m")
    print(f"vtol_states={states.tolist()}")
    for name, passed in checks.items():
        print(f"{name}={'PASS' if passed else 'FAIL'}")
    if not all(checks.values()):
        raise SystemExit("ulog_gate=FAIL")
    print("ulog_gate=PASS")


if __name__ == "__main__":
    main()
