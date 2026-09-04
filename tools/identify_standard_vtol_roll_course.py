#!/usr/bin/env python3
"""Identify L4 roll and coordinated-turn coefficients from a PX4 ULog."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from pyulog import ULog


def _dataset(ulog: ULog, name: str, multi_id: int = 0):
    return next(
        item.data for item in ulog.data_list
        if item.name == name and item.multi_id == multi_id
    )


def _interp(target_time, data, field):
    return np.interp(
        target_time,
        np.asarray(data["timestamp"], dtype=float) * 1.0e-6,
        np.asarray(data[field], dtype=float),
    )


def identify(path: Path) -> dict[str, float]:
    ulog = ULog(str(path))
    angular = _dataset(ulog, "vehicle_angular_velocity")
    servos = _dataset(ulog, "actuator_servos")
    airspeed = _dataset(ulog, "airspeed_validated")
    status = _dataset(ulog, "vehicle_status")
    attitude = _dataset(ulog, "vehicle_attitude")
    local = _dataset(ulog, "vehicle_local_position")

    angular_time = np.asarray(angular["timestamp"], dtype=float) * 1.0e-6
    nav_state = np.rint(_interp(angular_time, status, "nav_state"))
    speed = _interp(angular_time, airspeed, "calibrated_airspeed_m_s")
    aileron = 0.5 * (
        _interp(angular_time, servos, "control[1]")
        - _interp(angular_time, servos, "control[0]")
    )
    roll_rate = np.asarray(angular["xyz[0]"], dtype=float)
    roll_acceleration = np.asarray(angular["xyz_derivative[0]"], dtype=float)
    roll_mask = (
        (nav_state == 14)
        & (speed > 7.0)
        & np.isfinite(aileron)
        & np.isfinite(roll_acceleration)
        & (np.abs(roll_acceleration) < 5.0)
    )
    roll_regressor = np.column_stack((
        roll_rate[roll_mask],
        speed[roll_mask] ** 2 * aileron[roll_mask],
        np.ones(np.count_nonzero(roll_mask)),
    ))
    roll_output = roll_acceleration[roll_mask]
    roll_coefficients = np.linalg.lstsq(
        roll_regressor, roll_output, rcond=None
    )[0]
    roll_residual = roll_output - roll_regressor @ roll_coefficients
    roll_r2 = 1.0 - np.sum(roll_residual ** 2) / np.sum(
        (roll_output - np.mean(roll_output)) ** 2
    )

    local_time = np.asarray(local["timestamp"], dtype=float) * 1.0e-6
    # PX4 may log two samples with the same timestamp. Remove duplicates before
    # differentiating course so the reported result is deterministic.
    unique = np.r_[True, np.diff(local_time) > 0.0]
    local_time = local_time[unique]
    vx = np.asarray(local["vx"], dtype=float)[unique]
    vy = np.asarray(local["vy"], dtype=float)[unique]
    ground_speed = np.hypot(vx, vy)
    course = np.unwrap(np.arctan2(vy, vx))
    course_rate = np.gradient(course, local_time)
    quaternion = [
        _interp(local_time, attitude, f"q[{index}]") for index in range(4)
    ]
    qw, qx, qy, qz = quaternion
    roll = np.arctan2(
        2.0 * (qw * qx + qy * qz),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )
    local_nav_state = np.rint(_interp(local_time, status, "nav_state"))
    ideal_course_rate = 9.80665 * np.tan(roll) / np.maximum(ground_speed, 1.0)
    course_mask = (
        (local_nav_state == 14)
        & (ground_speed > 8.0)
        & (np.abs(course_rate) < 1.0)
        & (np.abs(roll) < 0.5)
    )
    ideal = ideal_course_rate[course_mask]
    measured = course_rate[course_mask]
    course_gain = float(np.dot(ideal, measured) / np.dot(ideal, ideal))
    course_residual = measured - course_gain * ideal

    return {
        "roll_samples": int(np.count_nonzero(roll_mask)),
        "roll_damping_coefficient": float(roll_coefficients[0]),
        "roll_surface_coefficient": float(roll_coefficients[1]),
        "roll_bias": float(roll_coefficients[2]),
        "roll_r2": float(roll_r2),
        "course_samples": int(np.count_nonzero(course_mask)),
        "coordinated_turn_gain": course_gain,
        "coordinated_turn_rmse": float(np.sqrt(np.mean(course_residual ** 2))),
        "coordinated_turn_correlation": float(np.corrcoef(ideal, measured)[0, 1]),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("ulog", type=Path)
    args = parser.parse_args()
    result = identify(args.ulog)
    print(f"ULOG={args.ulog}")
    print(
        "p_dot = "
        f"{result['roll_damping_coefficient']:.6f} p + "
        f"{result['roll_surface_coefficient']:.6f} V^2 delta_a + "
        f"{result['roll_bias']:.6f}"
    )
    print(
        f"roll_samples={result['roll_samples']} "
        f"roll_r2={result['roll_r2']:.4f}"
    )
    print(
        "chi_dot = "
        f"{result['coordinated_turn_gain']:.6f} g tan(phi) / V"
    )
    print(
        f"course_samples={result['course_samples']} "
        f"course_rmse={result['coordinated_turn_rmse']:.5f}rad/s "
        f"course_correlation={result['coordinated_turn_correlation']:.4f}"
    )


if __name__ == "__main__":
    main()
