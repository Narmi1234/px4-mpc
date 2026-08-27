#!/usr/bin/env python3
"""Verify a stock-PX4 Standard VTOL front/back transition shadow gate."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from pyulog import ULog


MC = 3
TRANSITION_TO_FW = 1
FW = 4
TRANSITION_TO_MC = 2
EXPECTED_SEQUENCE = (MC, TRANSITION_TO_FW, FW, TRANSITION_TO_MC, MC)


def get_data(ulog: ULog, name: str):
    matches = [item.data for item in ulog.data_list if item.name == name]
    if not matches:
        raise RuntimeError(f"ULog is missing {name}")
    return matches[0]


def parameter_at(ulog: ULog, name: str, timestamp_us: int):
    value = ulog.initial_parameters.get(name, "missing")
    for change_timestamp, change_name, change_value in ulog.changed_parameters:
        if change_name == name and int(change_timestamp) <= timestamp_us:
            value = change_value
    return value


def quaternion_angles(quaternion: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    qw, qx, qy, qz = quaternion.T
    roll = np.arctan2(
        2.0 * (qw * qx + qy * qz),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )
    pitch = np.arcsin(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0))
    return roll, pitch


def latest_complete_transition(vtol) -> tuple[int, int, int, list[int]]:
    timestamps = np.asarray(vtol["timestamp"], dtype=np.int64)
    states = np.asarray(vtol["vehicle_vtol_state"], dtype=int)
    change_indices = np.r_[0, np.flatnonzero(states[1:] != states[:-1]) + 1]
    compressed = states[change_indices]
    matches = []
    width = len(EXPECTED_SEQUENCE)
    for index in range(len(compressed) - width + 1):
        if tuple(compressed[index:index + width]) == EXPECTED_SEQUENCE:
            matches.append(index)
    if not matches:
        raise RuntimeError(
            "ULog has no complete MC->transition->FW->transition->MC sequence"
        )
    index = matches[-1]
    selected = change_indices[index:index + width]
    start = int(timestamps[selected[1]])
    fw_entry = int(timestamps[selected[2]])
    end = int(timestamps[selected[4]])
    return start, fw_entry, end, compressed.tolist()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("ulog", type=Path)
    parser.add_argument(
        "--gate-d",
        action="store_true",
        help="apply strict NMPC Gate D criteria and require external pusher",
    )
    parser.add_argument("--minimum-fw-entry-airspeed", type=float, default=None)
    parser.add_argument("--maximum-altitude-loss", type=float, default=None)
    parser.add_argument("--maximum-tilt-degrees", type=float, default=None)
    parser.add_argument("--maximum-transition-cycle-seconds", type=float, default=90.0)
    args = parser.parse_args()
    minimum_fw_entry_airspeed = (
        10.0 if args.minimum_fw_entry_airspeed is None
        else args.minimum_fw_entry_airspeed
    )
    maximum_altitude_loss = (
        2.0 if args.gate_d and args.maximum_altitude_loss is None
        else 12.0 if args.maximum_altitude_loss is None
        else args.maximum_altitude_loss
    )
    maximum_tilt_degrees = (
        20.0 if args.gate_d and args.maximum_tilt_degrees is None
        else 60.0 if args.maximum_tilt_degrees is None
        else args.maximum_tilt_degrees
    )

    ulog = ULog(
        str(args.ulog),
        message_name_filter_list=[
            "actuator_motors",
            "airspeed_validated",
            "vehicle_attitude",
            "vehicle_local_position",
            "vehicle_status",
            "vtol_vehicle_status",
        ],
    )
    vtol = get_data(ulog, "vtol_vehicle_status")
    position = get_data(ulog, "vehicle_local_position")
    attitude = get_data(ulog, "vehicle_attitude")
    airspeed = get_data(ulog, "airspeed_validated")
    motors = get_data(ulog, "actuator_motors")
    status = get_data(ulog, "vehicle_status")
    start, fw_entry, end, complete_state_history = latest_complete_transition(vtol)

    position_time = np.asarray(position["timestamp"], dtype=np.int64)
    position_mask = (position_time >= start) & (position_time <= end)
    z = np.asarray(position["z"], dtype=float)[position_mask]
    vx = np.asarray(position["vx"], dtype=float)[position_mask]
    vy = np.asarray(position["vy"], dtype=float)[position_mask]

    attitude_time = np.asarray(attitude["timestamp"], dtype=np.int64)
    attitude_mask = (attitude_time >= start) & (attitude_time <= end)
    quaternion = np.column_stack(
        [
            np.asarray(attitude[f"q[{index}]"], dtype=float)[attitude_mask]
            for index in range(4)
        ]
    )
    roll, pitch = quaternion_angles(quaternion)

    airspeed_time = np.asarray(airspeed["timestamp"], dtype=np.int64)
    calibrated = np.asarray(airspeed["calibrated_airspeed_m_s"], dtype=float)
    fw_entry_airspeed = float(np.interp(fw_entry, airspeed_time, calibrated))
    airspeed_mask = (airspeed_time >= start) & (airspeed_time <= end)
    peak_airspeed = float(np.nanmax(calibrated[airspeed_mask]))

    motor_time = np.asarray(motors["timestamp"], dtype=np.int64)
    motor_mask = (motor_time >= start) & (motor_time <= end)
    pusher = np.asarray(motors["control[4]"], dtype=float)[motor_mask]
    pusher = pusher[np.isfinite(pusher)]
    lift = np.column_stack(
        [
            np.asarray(motors[f"control[{index}]"], dtype=float)[motor_mask]
            for index in range(4)
        ]
    )
    lift = lift[np.isfinite(lift)]

    status_time = np.asarray(status["timestamp"], dtype=np.int64)
    status_mask = (status_time >= start) & (status_time <= end)
    failsafe = np.asarray(status["failsafe"], dtype=bool)[status_mask]

    if not len(z) or not len(quaternion) or not len(pusher) or not len(lift):
        raise RuntimeError("transition interval is missing required samples")

    duration = (end - start) * 1.0e-6
    altitude_loss = float(max(0.0, np.max(z - z[0])))
    altitude_excursion = float(np.max(np.abs(z - z[0])))
    maximum_tilt = float(
        np.degrees(np.max(np.maximum(np.abs(roll), np.abs(pitch))))
    )
    peak_ground_speed = float(np.max(np.hypot(vx, vy)))
    peak_pusher = float(np.max(pusher))
    minimum_lift = float(np.min(lift))
    maximum_lift = float(np.max(lift))
    external_pusher_enabled = parameter_at(ulog, "VT_EXT_PUSH_EN", start)
    external_pusher_max = parameter_at(ulog, "VT_EXT_PUSH_MAX", start)
    try:
        external_pusher_disabled = int(external_pusher_enabled) == 0
    except (TypeError, ValueError):
        external_pusher_disabled = False
    try:
        external_pusher_limit_correct = np.isclose(
            float(external_pusher_max), 0.60, atol=1.0e-4
        )
    except (TypeError, ValueError):
        external_pusher_limit_correct = False

    checks = {
        "complete_state_sequence": True,
        "cycle_duration": duration <= args.maximum_transition_cycle_seconds,
        "fw_entry_airspeed": (
            np.isfinite(fw_entry_airspeed)
            and fw_entry_airspeed >= minimum_fw_entry_airspeed
        ),
        "altitude_loss": altitude_loss <= maximum_altitude_loss,
        "tilt": maximum_tilt <= maximum_tilt_degrees,
        "no_failsafe": len(failsafe) > 0 and not np.any(failsafe),
        (
            "nmpc_external_pusher_enabled"
            if args.gate_d else "stock_px4_owns_pusher"
        ): (not external_pusher_disabled if args.gate_d else external_pusher_disabled),
        "pusher_active": peak_pusher >= 0.10,
        "pusher_bounded": peak_pusher <= 0.65 if args.gate_d else True,
        "external_pusher_limit": (
            external_pusher_limit_correct if args.gate_d else True
        ),
        "lift_blend_observed": minimum_lift <= 0.10 and maximum_lift >= 0.10,
    }

    print(f"transition_state_history={complete_state_history}")
    print(f"transition_cycle_duration={duration:.3f}s")
    print(f"fw_entry_airspeed={fw_entry_airspeed:.3f}m/s")
    print(f"peak_calibrated_airspeed={peak_airspeed:.3f}m/s")
    print(f"peak_ground_speed={peak_ground_speed:.3f}m/s")
    print(f"altitude_loss={altitude_loss:.3f}m")
    print(f"altitude_excursion={altitude_excursion:.3f}m")
    print(f"maximum_tilt={maximum_tilt:.2f}deg")
    print(f"pusher_peak={peak_pusher:.4f}")
    print(f"lift_motor_range=[{minimum_lift:.4f},{maximum_lift:.4f}]")
    print(f"VT_EXT_PUSH_EN={external_pusher_enabled}")
    print(f"VT_EXT_PUSH_MAX={external_pusher_max}")
    for name, passed in checks.items():
        print(f"{name}={'PASS' if passed else 'FAIL'}")
    result_name = "transition_gate_d_ulog" if args.gate_d else "transition_shadow_ulog_gate"
    if not all(checks.values()):
        raise SystemExit(f"{result_name}=FAIL")
    print(f"{result_name}=PASS")


if __name__ == "__main__":
    main()
