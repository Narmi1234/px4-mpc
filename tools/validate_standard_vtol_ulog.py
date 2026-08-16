#!/usr/bin/env python3
"""Compare a PX4 standard_vtol ULog with the Gazebo-derived plant model.

This is an offline plant-validation tool. It does not publish ROS topics and
cannot command the vehicle.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPOSITORY_ROOT / "px4_mpc"
sys.path.insert(0, str(PACKAGE_ROOT))

from px4_mpc.models.frames import (  # noqa: E402
    enu_to_ned,
    flu_to_frd,
    frd_to_flu,
    ned_to_enu,
    px4_quaternion_to_gazebo,
)
from px4_mpc.models.standard_vtol_gz_model import (  # noqa: E402
    StandardVtolGazeboModel,
    quaternion_to_rotation,
)


PHASE_NAMES = {1: "transition_to_fw", 2: "transition_to_mc", 3: "multicopter", 4: "fixed_wing"}


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate StandardVtolGazeboModel against a PX4 SITL ULog."
    )
    parser.add_argument("ulog", type=Path, help="Path to the .ulg file")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("results/standard_vtol_plant_validation"),
        help="Directory for report.md, samples.csv and plots",
    )
    parser.add_argument("--start", type=float, default=None, help="Start time in seconds after ULog start")
    parser.add_argument("--end", type=float, default=None, help="End time in seconds after ULog start")
    parser.add_argument("--rate", type=float, default=50.0, help="Validation sample rate in Hz")
    parser.add_argument(
        "--include-ground",
        action="store_true",
        help="Include landed samples; normally only in-air samples are evaluated",
    )
    parser.add_argument("--no-plots", action="store_true", help="Skip PNG plot generation")
    return parser.parse_args()


def require_dependencies():
    try:
        from pyulog import ULog
    except ImportError as error:
        raise SystemExit(
            "Missing pyulog. Install validation dependencies with: "
            "python3 -m pip install --user pyulog matplotlib"
        ) from error
    return ULog


def dataset(ulog, name: str, multi_id: int = 0):
    matches = [item for item in ulog.data_list if item.name == name and item.multi_id == multi_id]
    if not matches:
        raise RuntimeError(f"ULog is missing required dataset '{name}' instance {multi_id}")
    return matches[0]


def optional_dataset(ulog, name: str, multi_id: int = 0):
    matches = [item for item in ulog.data_list if item.name == name and item.multi_id == multi_id]
    return matches[0] if matches else None


def seconds(data, origin_us: float) -> np.ndarray:
    return (np.asarray(data["timestamp"], dtype=float) - origin_us) * 1.0e-6


def interpolate(data, field: str, timeline: np.ndarray, origin_us: float) -> np.ndarray:
    return np.interp(timeline, seconds(data, origin_us), np.asarray(data[field], dtype=float))


def interpolate_columns(data, prefix: str, count: int, timeline: np.ndarray, origin_us: float) -> np.ndarray:
    return np.column_stack(
        [interpolate(data, f"{prefix}[{index}]", timeline, origin_us) for index in range(count)]
    )


def nearest(data, field: str, timeline: np.ndarray, origin_us: float) -> np.ndarray:
    source_time = seconds(data, origin_us)
    indices = np.searchsorted(source_time, timeline, side="left")
    indices = np.clip(indices, 0, len(source_time) - 1)
    previous = np.maximum(indices - 1, 0)
    use_previous = np.abs(timeline - source_time[previous]) < np.abs(source_time[indices] - timeline)
    indices[use_previous] = previous[use_previous]
    return np.asarray(data[field])[indices]


def moving_average(values: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return values.copy()
    kernel = np.ones(window) / window
    return np.column_stack(
        [np.convolve(values[:, index], kernel, mode="same") for index in range(values.shape[1])]
    )


def servo_angles(ulog, servo_output, timeline: np.ndarray, origin_us: float) -> np.ndarray:
    raw = interpolate_columns(servo_output.data, "output", 3, timeline, origin_us)
    result = np.zeros_like(raw)
    parameters = ulog.initial_parameters
    for index in range(3):
        output_min = float(parameters.get(f"SIM_GZ_SV_MIN{index + 1}", 0.0))
        output_max = float(parameters.get(f"SIM_GZ_SV_MAX{index + 1}", 1000.0))
        angle_min = np.deg2rad(float(parameters.get(f"SIM_GZ_SV_MINA{index + 1}", -45.0)))
        angle_max = np.deg2rad(float(parameters.get(f"SIM_GZ_SV_MAXA{index + 1}", 45.0)))
        normalized = np.clip((raw[:, index] - output_min) / (output_max - output_min), 0.0, 1.0)
        result[:, index] = angle_min + normalized * (angle_max - angle_min)
    return result


def filter_rotor_speeds(model: StandardVtolGazeboModel, targets: np.ndarray, dt: float) -> np.ndarray:
    speeds = np.zeros_like(targets)
    speeds[0] = np.maximum(targets[0], 0.0)
    for sample in range(1, len(targets)):
        for motor_index, motor in enumerate(model.motors):
            target = max(0.0, targets[sample, motor_index])
            previous = speeds[sample - 1, motor_index]
            tau = motor.time_constant_up if target > previous else motor.time_constant_down
            alpha = np.exp(-dt / tau)
            speeds[sample, motor_index] = alpha * previous + (1.0 - alpha) * target
    return speeds


def calculate_metrics(measured: np.ndarray, predicted: np.ndarray, mask: np.ndarray) -> dict[str, np.ndarray]:
    residual = predicted[mask] - measured[mask]
    if len(residual) == 0:
        missing = np.full(3, np.nan)
        return {"rmse": missing, "mae": missing, "bias": missing, "nrmse": missing}
    measured_rms = np.sqrt(np.mean(measured[mask] ** 2, axis=0))
    rmse = np.sqrt(np.mean(residual**2, axis=0))
    nrmse = np.full(3, np.nan)
    excited_axes = measured_rms > 0.10
    nrmse[excited_axes] = 100.0 * rmse[excited_axes] / measured_rms[excited_axes]
    return {
        "rmse": rmse,
        "mae": np.mean(np.abs(residual), axis=0),
        "bias": np.mean(residual, axis=0),
        "nrmse": nrmse,
    }


def metric_row(values: np.ndarray) -> str:
    return " | ".join(f"{value:.3f}" if np.isfinite(value) else "n/a" for value in values)


def write_report(
    path: Path,
    ulog_path: Path,
    timeline: np.ndarray,
    valid_mask: np.ndarray,
    phase: np.ndarray,
    acceleration_metrics: dict[str, np.ndarray],
    angular_metrics: dict[str, np.ndarray],
    phase_metrics: dict[int, tuple[int, dict[str, np.ndarray]]],
    state_source: str,
) -> None:
    lines = [
        "# Standard VTOL plant validation report",
        "",
        f"- ULog: `{ulog_path.resolve()}`",
        f"- Evaluated interval: `{timeline[0]:.2f} s` to `{timeline[-1]:.2f} s`",
        f"- Accepted airborne samples: `{int(np.sum(valid_mask))}` / `{len(timeline)}`",
        f"- Translational state source: `{state_source}`",
        "- Linear axes: PX4 NED `[North, East, Down]`",
        "- Angular axes: PX4 FRD `[roll, pitch, yaw]`",
        "",
        "## Overall residuals (prediction - measurement)",
        "",
        "### Linear acceleration [m/s^2]",
        "",
        "| Metric | North | East | Down |",
        "|---|---:|---:|---:|",
        f"| RMSE | {metric_row(acceleration_metrics['rmse'])} |",
        f"| MAE | {metric_row(acceleration_metrics['mae'])} |",
        f"| Bias | {metric_row(acceleration_metrics['bias'])} |",
        f"| NRMSE [%] | {metric_row(acceleration_metrics['nrmse'])} |",
        "",
        "### Angular acceleration [rad/s^2]",
        "",
        "| Metric | Roll | Pitch | Yaw |",
        "|---|---:|---:|---:|",
        f"| RMSE | {metric_row(angular_metrics['rmse'])} |",
        f"| MAE | {metric_row(angular_metrics['mae'])} |",
        f"| Bias | {metric_row(angular_metrics['bias'])} |",
        f"| NRMSE [%] | {metric_row(angular_metrics['nrmse'])} |",
        "",
        "## Linear-acceleration RMSE by VTOL phase",
        "",
        "| Phase | Samples | RMSE N | RMSE E | RMSE D | NRMSE N | NRMSE E | NRMSE D |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for phase_id, (count, metrics) in phase_metrics.items():
        lines.append(
            f"| {PHASE_NAMES.get(phase_id, str(phase_id))} | {count} | "
            f"{metric_row(metrics['rmse'])} | {metric_row(metrics['nrmse'])} |"
        )
    lines.extend(
        [
            "",
            "## Interpretation",
            "",
            "This is a one-step force/moment comparison, not a long open-loop rollout. "
            "Large constant bias usually indicates a frame, mass, trim, or actuator mapping error. "
            "Error that grows mainly with airspeed points to the aerodynamic model. Large short spikes "
            "around commands can indicate timing or unmodelled actuator dynamics.",
            "",
            "The initial targets are correct signs on all axes and hover acceleration bias below "
            "`0.1 m/s^2`. A forward-flight NRMSE below approximately `15%` is a later target after "
            "the comparison windows and aerodynamic parameters have been reviewed. NRMSE is reported "
            "as `n/a` on an axis whose measured RMS is too small to be meaningfully normalized. Do not "
            "tune NMPC weights to compensate for a systematic plant-model error.",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_csv(
    path: Path,
    timeline: np.ndarray,
    valid: np.ndarray,
    phase: np.ndarray,
    airspeed: np.ndarray,
    measured_accel: np.ndarray,
    predicted_accel: np.ndarray,
    measured_alpha: np.ndarray,
    predicted_alpha: np.ndarray,
    targets: np.ndarray,
    servo: np.ndarray,
    velocity_body: np.ndarray,
    measured_aero_force_body: np.ndarray,
    sdf_aero_force_body: np.ndarray,
) -> None:
    header = [
        "time_s", "valid", "vtol_phase", "airspeed_mps",
        "measured_ax_n", "measured_ay_e", "measured_az_d",
        "predicted_ax_n", "predicted_ay_e", "predicted_az_d",
        "measured_alpha_roll", "measured_alpha_pitch", "measured_alpha_yaw",
        "predicted_alpha_roll", "predicted_alpha_pitch", "predicted_alpha_yaw",
        "motor_0_rad_s", "motor_1_rad_s", "motor_2_rad_s", "motor_3_rad_s", "pusher_rad_s",
        "left_elevon_rad", "right_elevon_rad", "elevator_rad",
        "velocity_body_x", "velocity_body_y", "velocity_body_z",
        "measured_aero_force_body_x", "measured_aero_force_body_y",
        "measured_aero_force_body_z", "sdf_aero_force_body_x",
        "sdf_aero_force_body_y", "sdf_aero_force_body_z",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        for index in range(len(timeline)):
            writer.writerow(
                [timeline[index], int(valid[index]), int(phase[index]), airspeed[index]]
                + measured_accel[index].tolist()
                + predicted_accel[index].tolist()
                + measured_alpha[index].tolist()
                + predicted_alpha[index].tolist()
                + targets[index].tolist()
                + servo[index].tolist()
                + velocity_body[index].tolist()
                + measured_aero_force_body[index].tolist()
                + sdf_aero_force_body[index].tolist()
            )


def write_plots(
    output: Path,
    timeline: np.ndarray,
    valid: np.ndarray,
    measured_accel: np.ndarray,
    predicted_accel: np.ndarray,
    measured_alpha: np.ndarray,
    predicted_alpha: np.ndarray,
    airspeed: np.ndarray,
    targets: np.ndarray,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plot_time = timeline[valid]
    linear_labels = ("North", "East", "Down")
    angular_labels = ("Roll", "Pitch", "Yaw")

    figure, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for index, axis in enumerate(axes):
        axis.plot(plot_time, measured_accel[valid, index], label="ULog", linewidth=1.0)
        axis.plot(plot_time, predicted_accel[valid, index], label="model", linewidth=1.0)
        axis.set_ylabel(f"{linear_labels[index]} [m/s²]")
        axis.grid(True)
    axes[0].legend()
    axes[-1].set_xlabel("ULog time [s]")
    figure.tight_layout()
    figure.savefig(output / "linear_acceleration.png", dpi=150)
    plt.close(figure)

    figure, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for index, axis in enumerate(axes):
        axis.plot(plot_time, measured_alpha[valid, index], label="ULog", linewidth=1.0)
        axis.plot(plot_time, predicted_alpha[valid, index], label="model", linewidth=1.0)
        axis.set_ylabel(f"{angular_labels[index]} [rad/s²]")
        axis.grid(True)
    axes[0].legend()
    axes[-1].set_xlabel("ULog time [s]")
    figure.tight_layout()
    figure.savefig(output / "angular_acceleration.png", dpi=150)
    plt.close(figure)

    figure, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    axes[0].plot(timeline, airspeed)
    axes[0].set_ylabel("True airspeed [m/s]")
    axes[0].grid(True)
    for index, label in enumerate(("motor 0", "motor 1", "motor 2", "motor 3", "pusher")):
        axes[1].plot(timeline, targets[:, index], label=label)
    axes[1].set_ylabel("Command [rad/s]")
    axes[1].set_xlabel("ULog time [s]")
    axes[1].legend(ncol=5)
    axes[1].grid(True)
    figure.tight_layout()
    figure.savefig(output / "airspeed_and_motors.png", dpi=150)
    plt.close(figure)


def main() -> int:
    arguments = parse_arguments()
    if arguments.rate <= 0.0:
        raise SystemExit("--rate must be positive")
    if not arguments.ulog.is_file():
        raise SystemExit(f"ULog does not exist: {arguments.ulog}")

    ULog = require_dependencies()
    names = [
        "actuator_outputs", "vehicle_local_position", "vehicle_attitude",
        "vehicle_angular_velocity", "airspeed_validated", "vtol_vehicle_status",
        "vehicle_land_detected", "vehicle_local_position_groundtruth",
        "vehicle_attitude_groundtruth",
    ]
    ulog = ULog(str(arguments.ulog), message_name_filter_list=names)
    motor_output = dataset(ulog, "actuator_outputs", 0)
    servo_output = dataset(ulog, "actuator_outputs", 1)
    local_position_groundtruth = optional_dataset(
        ulog, "vehicle_local_position_groundtruth"
    )
    attitude_groundtruth = optional_dataset(ulog, "vehicle_attitude_groundtruth")
    if local_position_groundtruth is not None and attitude_groundtruth is not None:
        local_position = local_position_groundtruth
        attitude = attitude_groundtruth
        state_source = "Gazebo ground truth"
    else:
        local_position = dataset(ulog, "vehicle_local_position")
        attitude = dataset(ulog, "vehicle_attitude")
        state_source = "PX4 estimator (ground truth unavailable)"
    angular_velocity = dataset(ulog, "vehicle_angular_velocity")
    airspeed_data = dataset(ulog, "airspeed_validated")
    vtol_status = dataset(ulog, "vtol_vehicle_status")
    land_detected = optional_dataset(ulog, "vehicle_land_detected")

    all_starts = [item.data["timestamp"][0] for item in (motor_output, servo_output, local_position, attitude, angular_velocity)]
    all_ends = [item.data["timestamp"][-1] for item in (motor_output, servo_output, local_position, attitude, angular_velocity)]
    origin_us = float(min(all_starts))
    start_s = max((value - origin_us) * 1.0e-6 for value in all_starts)
    end_s = min((value - origin_us) * 1.0e-6 for value in all_ends)
    if arguments.start is not None:
        start_s = max(start_s, arguments.start)
    if arguments.end is not None:
        end_s = min(end_s, arguments.end)
    if end_s <= start_s:
        raise SystemExit(f"No common data interval after filters: start={start_s:.3f}, end={end_s:.3f}")

    dt = 1.0 / arguments.rate
    timeline = np.arange(start_s, end_s, dt)
    targets = interpolate_columns(motor_output.data, "output", 5, timeline, origin_us)
    servo = servo_angles(ulog, servo_output, timeline, origin_us)
    position_ned = np.column_stack(
        [interpolate(local_position.data, field, timeline, origin_us) for field in ("x", "y", "z")]
    )
    velocity_ned = np.column_stack(
        [interpolate(local_position.data, field, timeline, origin_us) for field in ("vx", "vy", "vz")]
    )
    measured_acceleration_ned = np.column_stack(
        [interpolate(local_position.data, field, timeline, origin_us) for field in ("ax", "ay", "az")]
    )
    quaternions_ned_frd = interpolate_columns(attitude.data, "q", 4, timeline, origin_us)
    quaternions_ned_frd /= np.linalg.norm(quaternions_ned_frd, axis=1, keepdims=True)
    omega_frd = interpolate_columns(angular_velocity.data, "xyz", 3, timeline, origin_us)
    measured_alpha_frd = interpolate_columns(
        angular_velocity.data, "xyz_derivative", 3, timeline, origin_us
    )
    airspeed = interpolate(airspeed_data.data, "true_airspeed_m_s", timeline, origin_us)
    phase = nearest(vtol_status.data, "vehicle_vtol_state", timeline, origin_us).astype(int)

    model = StandardVtolGazeboModel()
    rotor_speeds = filter_rotor_speeds(model, targets, dt)
    predicted_acceleration_ned = np.zeros((len(timeline), 3))
    predicted_alpha_frd = np.zeros((len(timeline), 3))
    rotations_world_body = np.zeros((len(timeline), 3, 3))
    velocity_body = np.zeros((len(timeline), 3))
    motor_force_body = np.zeros((len(timeline), 3))
    sdf_aero_force_body = np.zeros((len(timeline), 3))
    for index in range(len(timeline)):
        state = np.zeros(18)
        state[0:3] = ned_to_enu(position_ned[index])
        state[3:6] = ned_to_enu(velocity_ned[index])
        state[6:10] = px4_quaternion_to_gazebo(quaternions_ned_frd[index])
        state[10:13] = frd_to_flu(omega_frd[index])
        state[13:18] = rotor_speeds[index]
        control = np.zeros(8)
        control[5:8] = servo[index]
        rotation = quaternion_to_rotation(state[6:10])
        rotations_world_body[index] = rotation
        velocity_body[index] = rotation.T @ state[3:6]
        motor_force_body[index], _ = model.motor_wrench(
            rotor_speeds[index], velocity_body[index], state[10:13]
        )
        sdf_aero_force_body[index], _ = model.aerodynamic_wrench(
            servo[index], velocity_body[index], state[10:13]
        )
        derivative = model.derivative(state, control)
        predicted_acceleration_ned[index] = enu_to_ned(derivative[3:6])
        predicted_alpha_frd[index] = flu_to_frd(derivative[10:13])

    # PX4 acceleration estimates are filtered. Apply a short matching smoother
    # so high-frequency estimator noise does not dominate the plant comparison.
    window = max(1, int(round(0.10 * arguments.rate)))
    measured_acceleration_ned = moving_average(measured_acceleration_ned, window)
    measured_alpha_frd = moving_average(measured_alpha_frd, window)
    predicted_acceleration_ned = moving_average(predicted_acceleration_ned, window)
    predicted_alpha_frd = moving_average(predicted_alpha_frd, window)

    gravity_world = np.array([0.0, 0.0, -model.gravity])
    measured_acceleration_world = np.column_stack(
        (
            measured_acceleration_ned[:, 1],
            measured_acceleration_ned[:, 0],
            -measured_acceleration_ned[:, 2],
        )
    )
    measured_aero_force_body = np.zeros((len(timeline), 3))
    for index in range(len(timeline)):
        total_force_body = (
            model.mass
            * rotations_world_body[index].T
            @ (measured_acceleration_world[index] - gravity_world)
        )
        measured_aero_force_body[index] = total_force_body - motor_force_body[index]

    finite = np.all(
        np.isfinite(
            np.column_stack(
                [measured_acceleration_ned, predicted_acceleration_ned, measured_alpha_frd, predicted_alpha_frd]
            )
        ),
        axis=1,
    )
    # In fixed-wing flight the four lift motors are intentionally stopped, so
    # requiring lift-motor activity would discard the most important segment.
    # The landed flag below is the primary ground filter; actuator activity
    # only removes quiet samples before startup or after shutdown.
    actuators_active = np.max(targets, axis=1) > 100.0
    valid = finite & actuators_active
    if land_detected is not None and not arguments.include_ground:
        landed = nearest(land_detected.data, "landed", timeline, origin_us).astype(bool)
        valid &= ~landed
    if np.sum(valid) < 10:
        raise SystemExit(
            "Fewer than 10 valid airborne samples. Fly and land normally, or inspect with --include-ground."
        )

    acceleration_metrics = calculate_metrics(measured_acceleration_ned, predicted_acceleration_ned, valid)
    angular_metrics = calculate_metrics(measured_alpha_frd, predicted_alpha_frd, valid)
    phase_metrics = {}
    for phase_id in sorted(PHASE_NAMES):
        phase_mask = valid & (phase == phase_id)
        if np.any(phase_mask):
            phase_metrics[phase_id] = (
                int(np.sum(phase_mask)),
                calculate_metrics(measured_acceleration_ned, predicted_acceleration_ned, phase_mask),
            )

    arguments.output.mkdir(parents=True, exist_ok=True)
    write_report(
        arguments.output / "report.md", arguments.ulog, timeline, valid, phase,
        acceleration_metrics, angular_metrics, phase_metrics,
        state_source,
    )
    write_csv(
        arguments.output / "samples.csv", timeline, valid, phase, airspeed,
        measured_acceleration_ned, predicted_acceleration_ned,
        measured_alpha_frd, predicted_alpha_frd, targets, servo,
        velocity_body, measured_aero_force_body, sdf_aero_force_body,
    )
    if not arguments.no_plots:
        write_plots(
            arguments.output, timeline, valid, measured_acceleration_ned,
            predicted_acceleration_ned, measured_alpha_frd, predicted_alpha_frd,
            airspeed, targets,
        )

    print(f"Validated {int(np.sum(valid))} airborne samples")
    print("Linear acceleration RMSE [N,E,D]:", metric_row(acceleration_metrics["rmse"]))
    print("Angular acceleration RMSE [roll,pitch,yaw]:", metric_row(angular_metrics["rmse"]))
    print(f"Report: {(arguments.output / 'report.md').resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
