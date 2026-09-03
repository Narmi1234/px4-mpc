#!/usr/bin/env python3
"""Extract time-aligned PX4 rate-loop samples from a Standard VTOL ULog.

This tool is strictly offline: it only reads a ULog and writes a CSV/report.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np
from pyulog import ULog


VTOL_NAMES = {
    1: "transition_to_fw",
    2: "transition_to_mc",
    3: "multicopter",
    4: "fixed_wing",
}


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ulog", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument(
        "--external-allocation",
        action="store_true",
        help=(
            "infer the applied lift allocation from final lift-motor commands "
            "and the collective setpoint; use for NMPC-owned MC allocation logs"
        ),
    )
    return parser.parse_args()


def dataset(ulog: ULog, name: str, multi_id: int = 0):
    matches = [
        item for item in ulog.data_list
        if item.name == name and item.multi_id == multi_id
    ]
    if not matches:
        raise RuntimeError(f"ULog is missing {name} instance {multi_id}")
    return matches[0].data


def seconds(data, origin_us: float) -> np.ndarray:
    return (np.asarray(data["timestamp"], dtype=float) - origin_us) * 1.0e-6


def interpolate(data, field: str, timeline: np.ndarray, origin_us: float):
    return np.interp(
        timeline, seconds(data, origin_us), np.asarray(data[field], dtype=float)
    )


def columns(data, prefix: str, count: int, timeline, origin_us):
    return np.column_stack(
        [interpolate(data, f"{prefix}[{i}]", timeline, origin_us) for i in range(count)]
    )


def nearest(data, field: str, timeline: np.ndarray, origin_us: float):
    source_time = seconds(data, origin_us)
    indices = np.searchsorted(source_time, timeline, side="left")
    indices = np.clip(indices, 0, len(source_time) - 1)
    previous = np.maximum(indices - 1, 0)
    use_previous = (
        np.abs(timeline - source_time[previous])
        < np.abs(source_time[indices] - timeline)
    )
    indices[use_previous] = previous[use_previous]
    return np.asarray(data[field])[indices]


def phase_zone(vtol_state: int, lift_fraction: float | None = None) -> str:
    # The custom PX4 allocation interface deliberately leaves vehicle_vtol_state
    # in MC while NMPC transfers lift authority.  Label that interval as blend
    # when a measured allocation proxy is available.
    if (
        vtol_state == 3
        and lift_fraction is not None
        and np.isfinite(lift_fraction)
        and lift_fraction < 0.98
    ):
        return "blend"
    if vtol_state == 3:
        return "mc"
    if vtol_state == 4:
        return "fw"
    if vtol_state in (1, 2):
        return "blend"
    return "other"


def quaternion_to_rotation(quaternion: np.ndarray) -> np.ndarray:
    """PX4 q(NED->FRD): return rotation from body FRD to world NED."""
    w, x, y, z = quaternion / np.linalg.norm(quaternion)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def main() -> int:
    options = arguments()
    if options.rate <= 0.0:
        raise SystemExit("--rate must be positive")
    if not options.ulog.is_file():
        raise SystemExit(f"ULog does not exist: {options.ulog}")

    names = [
        "actuator_motors",
        "actuator_outputs",
        "actuator_servos",
        "airspeed_validated",
        "gain_compression",
        "rate_ctrl_status",
        "vehicle_attitude",
        "vehicle_angular_velocity",
        "vehicle_angular_velocity_groundtruth",
        "vehicle_land_detected",
        "vehicle_local_position",
        "vehicle_rates_setpoint",
        "vehicle_status",
        "vehicle_torque_setpoint",
        "vtol_vehicle_status",
    ]
    ulog = ULog(str(options.ulog), message_name_filter_list=names)
    rates = dataset(ulog, "vehicle_rates_setpoint")
    angular = dataset(ulog, "vehicle_angular_velocity")
    angular_groundtruth = dataset(ulog, "vehicle_angular_velocity_groundtruth")
    airspeed_data = dataset(ulog, "airspeed_validated")
    vtol = dataset(ulog, "vtol_vehicle_status")
    motors = dataset(ulog, "actuator_motors")
    motor_outputs = dataset(ulog, "actuator_outputs", 0)
    servo_outputs = dataset(ulog, "actuator_outputs", 1)
    servos = dataset(ulog, "actuator_servos")
    attitude = dataset(ulog, "vehicle_attitude")
    position = dataset(ulog, "vehicle_local_position")
    torque_mc = dataset(ulog, "vehicle_torque_setpoint", 0)
    torque_fw = dataset(ulog, "vehicle_torque_setpoint", 1)
    rate_status_mc = dataset(ulog, "rate_ctrl_status", 0)
    rate_status_fw = dataset(ulog, "rate_ctrl_status", 1)
    gain_compression = dataset(ulog, "gain_compression")
    status = dataset(ulog, "vehicle_status")
    landed_data = dataset(ulog, "vehicle_land_detected")

    required = (
        rates, angular, angular_groundtruth, airspeed_data, vtol, motors, motor_outputs, servo_outputs,
        servos, attitude, position,
        torque_mc, torque_fw, rate_status_mc, rate_status_fw, gain_compression,
        status, landed_data,
    )
    origin_us = float(min(data["timestamp"][0] for data in required))
    start = max(seconds(data, origin_us)[0] for data in required)
    end = min(seconds(data, origin_us)[-1] for data in required)
    dt = 1.0 / options.rate
    timeline = np.arange(start, end, dt)
    if len(timeline) < 10:
        raise SystemExit("No sufficiently long common interval in ULog")

    rate_sp = np.column_stack(
        [interpolate(rates, axis, timeline, origin_us) for axis in ("roll", "pitch", "yaw")]
    )
    omega = columns(angular, "xyz", 3, timeline, origin_us)
    omega_dot_logged = columns(angular, "xyz_derivative", 3, timeline, origin_us)
    omega_groundtruth = columns(angular_groundtruth, "xyz", 3, timeline, origin_us)
    # Gazebo ground-truth message currently leaves xyz_derivative at zero.
    # Derive it from the high-rate ground-truth angular velocity instead.
    omega_dot_groundtruth = np.gradient(omega_groundtruth, dt, axis=0)
    smoothing_window = 3
    kernel = np.ones(smoothing_window) / smoothing_window
    omega_dot_groundtruth = np.column_stack([
        np.convolve(omega_dot_groundtruth[:, axis], kernel, mode="same")
        for axis in range(3)
    ])
    thrust_sp = columns(rates, "thrust_body", 3, timeline, origin_us)
    cas = interpolate(airspeed_data, "calibrated_airspeed_m_s", timeline, origin_us)
    # PX4 FixedwingRateControl uses a 1 s AlphaFilter before torque scaling.
    fw_filtered_airspeed = np.zeros(len(timeline))
    filter_alpha = dt / (1.0 + dt)
    for i, value in enumerate(cas):
        sample = max(0.5, float(value)) if np.isfinite(value) else 15.0
        previous = fw_filtered_airspeed[i - 1] if i else 0.0
        fw_filtered_airspeed[i] = previous + filter_alpha * (sample - previous)
    vtol_state = nearest(vtol, "vehicle_vtol_state", timeline, origin_us).astype(int)
    motor_control = columns(motors, "control", 5, timeline, origin_us)
    motor_target_speed = columns(motor_outputs, "output", 5, timeline, origin_us)
    servo_raw = columns(servo_outputs, "output", 3, timeline, origin_us)
    servo_angle = np.zeros_like(servo_raw)
    for index in range(3):
        output_min = float(ulog.initial_parameters.get(f"SIM_GZ_SV_MIN{index + 1}", 0.0))
        output_max = float(ulog.initial_parameters.get(f"SIM_GZ_SV_MAX{index + 1}", 1000.0))
        angle_min = np.deg2rad(float(ulog.initial_parameters.get(f"SIM_GZ_SV_MINA{index + 1}", -45.0)))
        angle_max = np.deg2rad(float(ulog.initial_parameters.get(f"SIM_GZ_SV_MAXA{index + 1}", 45.0)))
        normalized = np.clip(
            (servo_raw[:, index] - output_min) / (output_max - output_min), 0.0, 1.0
        )
        servo_angle[:, index] = angle_min + normalized * (angle_max - angle_min)
    servo_control = columns(servos, "control", 3, timeline, origin_us)
    torque_mc_value = columns(torque_mc, "xyz", 3, timeline, origin_us)
    torque_fw_value = columns(torque_fw, "xyz", 3, timeline, origin_us)
    integral_fields = ("rollspeed_integ", "pitchspeed_integ", "yawspeed_integ")
    integral_mc = np.column_stack([
        interpolate(rate_status_mc, field, timeline, origin_us) for field in integral_fields
    ])
    integral_fw = np.column_stack([
        interpolate(rate_status_fw, field, timeline, origin_us) for field in integral_fields
    ])
    compression = columns(gain_compression, "compression_gains", 3, timeline, origin_us)
    quaternion = columns(attitude, "q", 4, timeline, origin_us)
    velocity_ned = np.column_stack(
        [interpolate(position, field, timeline, origin_us) for field in ("vx", "vy", "vz")]
    )
    velocity_body = np.zeros_like(velocity_ned)
    euler_pitch = np.zeros(len(timeline))
    for i in range(len(timeline)):
        rotation = quaternion_to_rotation(quaternion[i])
        velocity_body[i] = rotation.T @ velocity_ned[i]
        w, x, y, z = quaternion[i] / np.linalg.norm(quaternion[i])
        euler_pitch[i] = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    alpha = np.arctan2(velocity_body[:, 2], np.maximum(velocity_body[:, 0], 0.1))
    beta = np.arctan2(
        velocity_body[:, 1],
        np.sqrt(velocity_body[:, 0] ** 2 + velocity_body[:, 2] ** 2) + 1.0e-9,
    )
    arming_state = nearest(status, "arming_state", timeline, origin_us).astype(int)
    landed = nearest(landed_data, "landed", timeline, origin_us).astype(bool)

    finite_lift_count = np.sum(np.isfinite(motor_control[:, 0:4]), axis=1)
    lift_mean = np.divide(
        np.nansum(motor_control[:, 0:4], axis=1),
        finite_lift_count,
        out=np.full(len(timeline), np.nan),
        where=finite_lift_count > 0,
    )
    # PX4 publishes NaN for stopped/unallocated lift motors in FW. While armed
    # and airborne that means zero applied lift, not a missing measurement.
    stopped_lift = (finite_lift_count == 0) & (arming_state == 2) & (~landed)
    lift_mean[stopped_lift] = 0.0
    hover_mask = (
        (arming_state == 2)
        & (~landed)
        & (vtol_state == 3)
        & (np.abs(cas) < 3.0)
        & np.isfinite(lift_mean)
        & (lift_mean > 0.05)
    )
    hover_lift = float(np.nanmedian(lift_mean[hover_mask])) if np.any(hover_mask) else np.nan
    if np.isfinite(hover_lift) and hover_lift > 0.0:
        lift_fraction = np.clip(lift_mean / hover_lift, 0.0, 1.2)
    else:
        lift_fraction = np.full(len(timeline), np.nan)
    if options.external_allocation:
        # In the patched PX4 path the final lift command is approximately
        # collective * lambda.  This ratio reconstructs the *applied* lambda,
        # including PX4 slew limiting, without trusting the ROS command.  A
        # small collective floor avoids division while disarmed or stopped.
        collective = -thrust_sp[:, 2]
        allocation_mask = (
            (vtol_state == 3)
            & np.isfinite(lift_mean)
            & np.isfinite(collective)
            & (collective > 0.05)
        )
        lift_fraction = np.full(len(timeline), np.nan)
        lift_fraction[allocation_mask] = np.clip(
            lift_mean[allocation_mask] / collective[allocation_mask], 0.0, 1.2
        )
        lift_fraction[vtol_state == 4] = 0.0
    else:
        # Stock PX4 MC and FW states define the allocation endpoints exactly.
        # The hover-normalized motor ratio is used only during stock transition.
        lift_fraction[vtol_state == 3] = 1.0
        lift_fraction[vtol_state == 4] = 0.0

    zones = np.asarray([
        phase_zone(int(state), float(lift_fraction[index]))
        for index, state in enumerate(vtol_state)
    ])

    finite = (
        np.all(np.isfinite(rate_sp), axis=1)
        & np.all(np.isfinite(omega), axis=1)
        & np.all(np.isfinite(omega_groundtruth), axis=1)
        & (np.linalg.norm(omega_groundtruth, axis=1) < 10.0)
        & (np.linalg.norm(omega_dot_groundtruth, axis=1) < 50.0)
        & np.isfinite(cas)
    )
    valid = finite & (arming_state == 2) & (~landed) & np.isin(vtol_state, (1, 2, 3, 4))

    options.output.mkdir(parents=True, exist_ok=True)
    csv_path = options.output / "rate_samples.csv"
    fields = [
        "valid", "time_s", "timestamp_us", "vtol_state", "vtol_phase", "zone",
        "airspeed_mps", "fw_filtered_airspeed_mps", "lift_fraction_proxy", "lift_motor_mean", "pusher_actual",
        "collective_setpoint", "pitch_rad", "alpha_proxy_rad", "beta_proxy_rad",
        "velocity_body_x", "velocity_body_y", "velocity_body_z",
        "rate_sp_p", "rate_sp_q", "rate_sp_r",
        "omega_p", "omega_q", "omega_r",
        "omega_dot_p", "omega_dot_q", "omega_dot_r",
        "omega_gt_p", "omega_gt_q", "omega_gt_r",
        "omega_dot_gt_p", "omega_dot_gt_q", "omega_dot_gt_r",
        "mc_torque_x", "mc_torque_y", "mc_torque_z",
        "fw_torque_x", "fw_torque_y", "fw_torque_z",
        "mc_integrator_p", "mc_integrator_q", "mc_integrator_r",
        "fw_integrator_p", "fw_integrator_q", "fw_integrator_r",
        "fw_compression_p", "fw_compression_q", "fw_compression_r",
        "motor_0", "motor_1", "motor_2", "motor_3", "motor_4",
        "servo_0", "servo_1", "servo_2",
        "motor_target_speed_0", "motor_target_speed_1", "motor_target_speed_2",
        "motor_target_speed_3", "motor_target_speed_4",
        "surface_angle_0", "surface_angle_1", "surface_angle_2",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for i, time_s in enumerate(timeline):
            state = int(vtol_state[i])
            writer.writerow({
                "valid": int(valid[i]),
                "time_s": f"{time_s:.6f}",
                "timestamp_us": int(round(origin_us + time_s * 1.0e6)),
                "vtol_state": state,
                "vtol_phase": VTOL_NAMES.get(state, "unknown"),
                "zone": zones[i],
                "airspeed_mps": f"{cas[i]:.9g}",
                "fw_filtered_airspeed_mps": f"{fw_filtered_airspeed[i]:.9g}",
                "lift_fraction_proxy": f"{lift_fraction[i]:.9g}",
                "lift_motor_mean": f"{lift_mean[i]:.9g}",
                "pusher_actual": f"{motor_control[i, 4]:.9g}",
                "collective_setpoint": f"{-thrust_sp[i, 2]:.9g}",
                "pitch_rad": f"{euler_pitch[i]:.9g}",
                "alpha_proxy_rad": f"{alpha[i]:.9g}",
                "beta_proxy_rad": f"{beta[i]:.9g}",
                "velocity_body_x": f"{velocity_body[i, 0]:.9g}",
                "velocity_body_y": f"{velocity_body[i, 1]:.9g}",
                "velocity_body_z": f"{velocity_body[i, 2]:.9g}",
                "rate_sp_p": f"{rate_sp[i, 0]:.9g}",
                "rate_sp_q": f"{rate_sp[i, 1]:.9g}",
                "rate_sp_r": f"{rate_sp[i, 2]:.9g}",
                "omega_p": f"{omega[i, 0]:.9g}",
                "omega_q": f"{omega[i, 1]:.9g}",
                "omega_r": f"{omega[i, 2]:.9g}",
                "omega_dot_p": f"{omega_dot_logged[i, 0]:.9g}",
                "omega_dot_q": f"{omega_dot_logged[i, 1]:.9g}",
                "omega_dot_r": f"{omega_dot_logged[i, 2]:.9g}",
                "omega_gt_p": f"{omega_groundtruth[i, 0]:.9g}",
                "omega_gt_q": f"{omega_groundtruth[i, 1]:.9g}",
                "omega_gt_r": f"{omega_groundtruth[i, 2]:.9g}",
                "omega_dot_gt_p": f"{omega_dot_groundtruth[i, 0]:.9g}",
                "omega_dot_gt_q": f"{omega_dot_groundtruth[i, 1]:.9g}",
                "omega_dot_gt_r": f"{omega_dot_groundtruth[i, 2]:.9g}",
                "mc_torque_x": f"{torque_mc_value[i, 0]:.9g}",
                "mc_torque_y": f"{torque_mc_value[i, 1]:.9g}",
                "mc_torque_z": f"{torque_mc_value[i, 2]:.9g}",
                "fw_torque_x": f"{torque_fw_value[i, 0]:.9g}",
                "fw_torque_y": f"{torque_fw_value[i, 1]:.9g}",
                "fw_torque_z": f"{torque_fw_value[i, 2]:.9g}",
                "mc_integrator_p": f"{integral_mc[i, 0]:.9g}",
                "mc_integrator_q": f"{integral_mc[i, 1]:.9g}",
                "mc_integrator_r": f"{integral_mc[i, 2]:.9g}",
                "fw_integrator_p": f"{integral_fw[i, 0]:.9g}",
                "fw_integrator_q": f"{integral_fw[i, 1]:.9g}",
                "fw_integrator_r": f"{integral_fw[i, 2]:.9g}",
                "fw_compression_p": f"{compression[i, 0]:.9g}",
                "fw_compression_q": f"{compression[i, 1]:.9g}",
                "fw_compression_r": f"{compression[i, 2]:.9g}",
                "motor_0": f"{motor_control[i, 0]:.9g}",
                "motor_1": f"{motor_control[i, 1]:.9g}",
                "motor_2": f"{motor_control[i, 2]:.9g}",
                "motor_3": f"{motor_control[i, 3]:.9g}",
                "motor_4": f"{motor_control[i, 4]:.9g}",
                "servo_0": f"{servo_control[i, 0]:.9g}",
                "servo_1": f"{servo_control[i, 1]:.9g}",
                "servo_2": f"{servo_control[i, 2]:.9g}",
                "motor_target_speed_0": f"{motor_target_speed[i, 0]:.9g}",
                "motor_target_speed_1": f"{motor_target_speed[i, 1]:.9g}",
                "motor_target_speed_2": f"{motor_target_speed[i, 2]:.9g}",
                "motor_target_speed_3": f"{motor_target_speed[i, 3]:.9g}",
                "motor_target_speed_4": f"{motor_target_speed[i, 4]:.9g}",
                "surface_angle_0": f"{servo_angle[i, 0]:.9g}",
                "surface_angle_1": f"{servo_angle[i, 1]:.9g}",
                "surface_angle_2": f"{servo_angle[i, 2]:.9g}",
            })

    counts = {
        zone: int(np.count_nonzero(valid & (zones == zone)))
        for zone in ("mc", "blend", "fw")
    }
    report = [
        "# Standard VTOL rate dataset",
        "",
        f"- ULog: `{options.ulog.resolve()}`",
        f"- Sample rate: `{options.rate:.1f} Hz`",
        f"- Valid samples: `{int(np.count_nonzero(valid))}`",
        f"- Hover lift reference: `{hover_lift:.6f}`",
        (
            "- Allocation source: final lift-motor mean divided by collective "
            "setpoint (patched external-allocation path)."
            if options.external_allocation else
            "- `lift_fraction_proxy` is measured mean lift-motor command divided "
            "by the MC-hover median; it is diagnostic, not the future commanded lambda."
        ),
        "",
        "| Zone | Samples | Duration |",
        "|---|---:|---:|",
    ]
    for zone, count in counts.items():
        report.append(f"| {zone} | {count} | {count / options.rate:.2f} s |")
    (options.output / "extraction_report.md").write_text(
        "\n".join(report) + "\n", encoding="utf-8"
    )
    print(f"samples={csv_path.resolve()}")
    print(f"valid_samples={int(np.count_nonzero(valid))}")
    print("zone_samples=" + ",".join(f"{k}:{v}" for k, v in counts.items()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
