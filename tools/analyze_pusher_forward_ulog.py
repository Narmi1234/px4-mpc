#!/usr/bin/env python3
"""Verify Gate A from the actual PX4 actuator and flight-state ULog data."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from pyulog import ULog


OFFBOARD_NAV_STATE = 14
MC_VTOL_STATE = 3


def get_data(ulog: ULog, name: str):
    """Return the first required dataset by name."""
    matches = [item.data for item in ulog.data_list if item.name == name]
    if not matches:
        raise RuntimeError(f"ULog is missing {name}")
    return matches[0]


def optional_data(ulog: ULog, name: str):
    """Return an optional dataset or None."""
    matches = [item.data for item in ulog.data_list if item.name == name]
    return matches[0] if matches else None


def parameter_at(ulog: ULog, name: str, timestamp_us: int):
    """Return a parameter value after applying changes up to a timestamp."""
    value = ulog.initial_parameters.get(name, "missing")
    for change_timestamp, change_name, change_value in ulog.changed_parameters:
        if change_name == name and int(change_timestamp) <= timestamp_us:
            value = change_value
    return value


def last_offboard_interval(status) -> tuple[int, int]:
    """Return exact PX4 timestamps for the latest completed Offboard interval."""
    timestamps = np.asarray(status["timestamp"], dtype=np.int64)
    nav_timestamps = np.asarray(
        status.get("nav_state_timestamp", timestamps), dtype=np.int64
    )
    states = np.asarray(status["nav_state"], dtype=int)
    indices = np.flatnonzero(states == OFFBOARD_NAV_STATE)
    if not len(indices):
        raise RuntimeError("ULog contains no Offboard interval")
    first_index = int(indices[0])
    for index in indices[1:]:
        if int(index) > first_index and states[int(index) - 1] != OFFBOARD_NAV_STATE:
            first_index = int(index)
    end_index = int(indices[-1])
    exits = np.flatnonzero(
        (np.arange(len(states)) > end_index) & (states != OFFBOARD_NAV_STATE)
    )
    if not len(exits):
        raise RuntimeError("latest Offboard interval has no recorded exit")
    exit_index = int(exits[0])
    return int(nav_timestamps[first_index]), int(nav_timestamps[exit_index])


def quaternion_angles(quaternion: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return PX4 NED/FRD roll, pitch and yaw arrays."""
    qw, qx, qy, qz = quaternion.T
    roll = np.arctan2(
        2.0 * (qw * qx + qy * qz),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )
    pitch = np.arcsin(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0))
    yaw = np.arctan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    return roll, pitch, yaw


def main() -> None:
    """Calculate Gate A metrics and print a strict PASS/FAIL decision."""
    parser = argparse.ArgumentParser()
    parser.add_argument("ulog", type=Path)
    parser.add_argument("--minimum-duration", type=float, default=20.0)
    parser.add_argument("--minimum-forward-speed", type=float, default=2.5)
    parser.add_argument("--maximum-forward-speed", type=float, default=3.5)
    parser.add_argument("--maximum-final-speed", type=float, default=0.35)
    parser.add_argument("--maximum-altitude-error", type=float, default=0.30)
    parser.add_argument("--maximum-vertical-speed", type=float, default=0.75)
    parser.add_argument("--maximum-tilt-degrees", type=float, default=10.0)
    parser.add_argument("--maximum-cross-track", type=float, default=1.0)
    parser.add_argument("--expected-pusher-limit", type=float, default=0.10)
    parser.add_argument("--minimum-peak-airspeed", type=float, default=0.0)
    parser.add_argument("--require-airspeed", action="store_true")
    args = parser.parse_args()
    ulog = ULog(
        str(args.ulog),
        message_name_filter_list=[
            "actuator_motors",
            "airspeed_validated",
            "vehicle_attitude",
            "vehicle_local_position",
            "vehicle_rates_setpoint",
            "vehicle_status",
            "vtol_vehicle_status",
        ],
    )
    status = get_data(ulog, "vehicle_status")
    motors = get_data(ulog, "actuator_motors")
    position = get_data(ulog, "vehicle_local_position")
    attitude = get_data(ulog, "vehicle_attitude")
    vtol = get_data(ulog, "vtol_vehicle_status")
    airspeed = optional_data(ulog, "airspeed_validated")
    rates_setpoint = optional_data(ulog, "vehicle_rates_setpoint")
    start, end = last_offboard_interval(status)

    motor_time = np.asarray(motors["timestamp"], dtype=np.int64)
    motor_mask = (motor_time >= start) & (motor_time <= end)
    pusher = np.asarray(motors["control[4]"], dtype=float)[motor_mask]
    finite_pusher = pusher[np.isfinite(pusher)]
    final_pusher = np.asarray(motors["control[4]"], dtype=float)[
        (motor_time >= end - 1_000_000) & (motor_time <= end)
    ]
    finite_final_pusher = final_pusher[np.isfinite(final_pusher)]

    commanded_pusher = np.array([], dtype=float)
    commanded_collective = np.array([], dtype=float)
    if rates_setpoint is not None:
        rates_time = np.asarray(rates_setpoint["timestamp"], dtype=np.int64)
        rates_mask = (rates_time >= start) & (rates_time <= end)
        commanded_pusher = np.asarray(
            rates_setpoint["thrust_body[0]"], dtype=float
        )[rates_mask]
        commanded_pusher = commanded_pusher[np.isfinite(commanded_pusher)]
        commanded_collective = -np.asarray(
            rates_setpoint["thrust_body[2]"], dtype=float
        )[rates_mask]
        commanded_collective = commanded_collective[
            np.isfinite(commanded_collective)
        ]

    lift_motor_values = np.column_stack(
        [
            np.asarray(motors[f"control[{index}]"], dtype=float)[motor_mask]
            for index in range(4)
        ]
    ).ravel()
    lift_motor_values = lift_motor_values[np.isfinite(lift_motor_values)]

    position_time = np.asarray(position["timestamp"], dtype=np.int64)
    position_mask = (position_time >= start) & (position_time <= end)
    x = np.asarray(position["x"], dtype=float)[position_mask]
    y = np.asarray(position["y"], dtype=float)[position_mask]
    z = np.asarray(position["z"], dtype=float)[position_mask]
    vx = np.asarray(position["vx"], dtype=float)[position_mask]
    vy = np.asarray(position["vy"], dtype=float)[position_mask]
    # Use the derivative of the same vertical position used for altitude
    # error. This remained sign-consistent with Gazebo ground truth in the
    # live logs, while one EKF vz estimate briefly had the opposite sign.
    vz = np.asarray(position["z_deriv"], dtype=float)[position_mask]

    attitude_time = np.asarray(attitude["timestamp"], dtype=np.int64)
    attitude_mask = (attitude_time >= start) & (attitude_time <= end)
    quaternion = np.column_stack(
        [
            np.asarray(attitude[f"q[{index}]"], dtype=float)[attitude_mask]
            for index in range(4)
        ]
    )
    roll, pitch, yaw = quaternion_angles(quaternion)
    start_yaw = float(yaw[0])
    forward = np.array([np.cos(start_yaw), np.sin(start_yaw)])
    normal = np.array([-forward[1], forward[0]])
    velocity_xy = np.column_stack([vx, vy])
    displacement_xy = np.column_stack([x - x[0], y - y[0]])
    forward_speed = velocity_xy @ forward
    cross_track = np.abs(displacement_xy @ normal)

    vtol_time = np.asarray(vtol["timestamp"], dtype=np.int64)
    before = np.flatnonzero(vtol_time <= start)
    during = np.flatnonzero((vtol_time >= start) & (vtol_time <= end))
    relevant = np.unique(np.r_[before[-1:] if len(before) else [], during]).astype(int)
    vtol_states = np.asarray(vtol["vehicle_vtol_state"], dtype=int)[relevant]

    peak_airspeed = float("nan")
    airspeed_stream_ok = False
    if airspeed is not None:
        airspeed_time = np.asarray(airspeed["timestamp"], dtype=np.int64)
        airspeed_mask = (airspeed_time >= start) & (airspeed_time <= end)
        calibrated = np.asarray(airspeed["calibrated_airspeed_m_s"], dtype=float)[
            airspeed_mask
        ]
        calibrated = calibrated[np.isfinite(calibrated)]
        if len(calibrated):
            peak_airspeed = float(np.max(calibrated))
        selected_times = airspeed_time[airspeed_mask]
        if len(selected_times) >= 2:
            maximum_gap = float(np.max(np.diff(selected_times))) * 1.0e-6
            airspeed_stream_ok = (
                selected_times[0] <= start + 500_000
                and selected_times[-1] >= end - 500_000
                and maximum_gap <= 0.5
            )

    if not len(x) or not len(quaternion):
        raise RuntimeError("Offboard interval is missing required flight samples")
    duration = (end - start) * 1.0e-6
    peak_commanded_pusher = (
        float(np.max(commanded_pusher)) if len(commanded_pusher) else float("nan")
    )
    peak_pusher = (
        float(np.max(finite_pusher)) if len(finite_pusher) else float("nan")
    )
    final_pusher_value = (
        float(np.max(np.abs(finite_final_pusher)))
        if len(finite_final_pusher)
        # PX4 logs a stopped non-reversible motor channel as NaN. Treat that
        # as zero only when the same channel was demonstrably active earlier.
        else (0.0 if len(finite_pusher) else float("nan"))
    )
    peak_forward_speed = float(np.max(forward_speed))
    peak_horizontal_speed = float(np.max(np.linalg.norm(velocity_xy, axis=1)))
    final_horizontal_speed = float(np.hypot(vx[-1], vy[-1]))
    max_altitude_error = float(np.max(np.abs(z - z[0])))
    max_vertical_speed = float(np.max(np.abs(vz)))
    max_tilt = float(
        np.degrees(np.max(np.maximum(np.abs(roll), np.abs(pitch))))
    )
    max_cross_track = float(np.max(cross_track))
    pusher_enabled = parameter_at(ulog, "VT_EXT_PUSH_EN", start)
    pusher_max = parameter_at(ulog, "VT_EXT_PUSH_MAX", start)
    pusher_slew = parameter_at(ulog, "VT_EXT_PUSH_SLEW", start)
    try:
        pusher_enabled_ok = int(pusher_enabled) == 1
        pusher_max_ok = abs(
            float(pusher_max) - float(args.expected_pusher_limit)
        ) <= 0.005
    except (TypeError, ValueError):
        pusher_enabled_ok = False
        pusher_max_ok = False
    checks = {
        "duration": duration >= args.minimum_duration,
        "forward_speed": args.minimum_forward_speed
        <= peak_forward_speed
        <= args.maximum_forward_speed,
        "horizontal_speed": peak_horizontal_speed <= args.maximum_forward_speed,
        "final_speed": final_horizontal_speed <= args.maximum_final_speed,
        "pusher_parameter_enabled": pusher_enabled_ok,
        "pusher_parameter_max": pusher_max_ok,
        "pusher_command_received": 0.049
        <= peak_commanded_pusher
        <= args.expected_pusher_limit + 0.005,
        "pusher_peak": 0.049
        <= peak_pusher
        <= args.expected_pusher_limit + 0.005,
        "pusher_returned_zero": final_pusher_value <= 0.005,
        "altitude": max_altitude_error <= args.maximum_altitude_error,
        "vertical_speed": max_vertical_speed <= args.maximum_vertical_speed,
        "tilt": max_tilt <= args.maximum_tilt_degrees,
        "cross_track": max_cross_track <= args.maximum_cross_track,
        "airspeed": (
            not args.require_airspeed
            or (
                airspeed_stream_ok
                and np.isfinite(peak_airspeed)
                and peak_airspeed >= args.minimum_peak_airspeed
            )
        ),
        "mc_only": len(vtol_states) > 0 and np.all(vtol_states == MC_VTOL_STATE),
    }
    print(f"offboard_duration={duration:.3f}s")
    print(f"peak_forward_speed={peak_forward_speed:.3f}m/s")
    print(f"peak_horizontal_speed={peak_horizontal_speed:.3f}m/s")
    print(f"final_horizontal_speed={final_horizontal_speed:.3f}m/s")
    print(f"peak_calibrated_airspeed={peak_airspeed:.3f}m/s")
    print(f"peak_commanded_pusher={peak_commanded_pusher:.4f}")
    print(f"peak_pusher_actuator={peak_pusher:.4f}")
    print(f"final_pusher_actuator={final_pusher_value:.4f}")
    if len(commanded_collective):
        print(
            "commanded_collective_range="
            f"[{np.min(commanded_collective):.4f},{np.max(commanded_collective):.4f}]"
        )
    if len(lift_motor_values):
        print(
            "lift_motor_range="
            f"[{np.min(lift_motor_values):.4f},{np.max(lift_motor_values):.4f}]"
        )
    print(
        "px4_pusher_parameters="
        f"[enabled={pusher_enabled},max={pusher_max},slew={pusher_slew}]"
    )
    print(f"max_altitude_error={max_altitude_error:.3f}m")
    print(f"max_vertical_speed={max_vertical_speed:.3f}m/s")
    print(f"max_tilt={max_tilt:.2f}deg")
    print(f"max_cross_track={max_cross_track:.3f}m")
    print(f"vtol_states={vtol_states.tolist()}")
    for name, passed in checks.items():
        print(f"{name}={'PASS' if passed else 'FAIL'}")
    if not all(checks.values()):
        raise SystemExit("ulog_gate=FAIL")
    print("ulog_gate=PASS")


if __name__ == "__main__":
    main()
