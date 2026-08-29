#!/usr/bin/env python3
"""Replay angular acceleration through logged and predicted actuator paths."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "px4_mpc"))

from px4_mpc.models.frames import flu_to_frd, frd_to_flu  # noqa: E402
from px4_mpc.models.standard_vtol_rotational_model import StandardVtolTorqueInformedModel  # noqa: E402


AXES = ("p", "q", "r")
ZONES = ("mc", "blend", "fw")


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, nargs="+")
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def vector(row, prefix, axes=AXES):
    return np.asarray([float(row[f"{prefix}_{axis}"]) for axis in axes])


def metrics(measured, predicted):
    measured = np.asarray(measured)
    predicted = np.asarray(predicted)
    residual = predicted - measured
    denominator = np.sqrt(
        np.sum((measured - np.mean(measured, axis=0)) ** 2, axis=0)
        * np.sum((predicted - np.mean(predicted, axis=0)) ** 2, axis=0)
    )
    correlation = np.divide(
        np.sum(
            (measured - np.mean(measured, axis=0))
            * (predicted - np.mean(predicted, axis=0)), axis=0
        ),
        denominator, out=np.zeros(3), where=denominator > 1e-12,
    )
    excited = np.abs(measured) > 0.1
    sign_match = np.divide(
        np.sum((np.sign(measured) == np.sign(predicted)) & excited, axis=0),
        np.sum(excited, axis=0), out=np.zeros(3), where=np.sum(excited, axis=0) > 0,
    )
    centered_measured = measured - np.mean(measured, axis=0)
    centered_predicted = predicted - np.mean(predicted, axis=0)
    calibration_gain = np.divide(
        np.sum(centered_measured * centered_predicted, axis=0),
        np.sum(centered_predicted**2, axis=0),
        out=np.zeros(3), where=np.sum(centered_predicted**2, axis=0) > 1e-12,
    )
    return {
        "samples": len(measured),
        "rmse": np.sqrt(np.mean(residual**2, axis=0)),
        "correlation": correlation,
        "excited_sign_match": sign_match,
        "measured_std": np.std(measured, axis=0),
        "predicted_std": np.std(predicted, axis=0),
        "calibration_gain": calibration_gain,
    }


def main():
    options = arguments()
    model = StandardVtolTorqueInformedModel()
    grouped = {
        (path, zone): {"measured": [], "logged": [], "predicted": []}
        for path in ("logged_actuators", "predicted_controller_allocator")
        for zone in ZONES
    }
    for path in options.csv:
        with path.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
        logged_rotor_speeds = np.zeros(5)
        logged_surface_state = np.zeros(3)
        predicted_surface_state = np.zeros(3)
        previous_time = None
        for row in rows:
            if int(row["valid"]) != 1 or row["zone"] not in ZONES:
                continue
            zone = row["zone"]
            velocity_frd = vector(row, "velocity_body", ("x", "y", "z"))
            # GZBridge's vehicle_angular_velocity_groundtruth is Euler-angle
            # derivative, not body angular velocity. PX4's gyro-derived body
            # rate is therefore the correct signal for this rigid-body replay.
            omega_frd = vector(row, "omega")
            omega_dot_frd = vector(row, "omega_dot")
            state = model.hover_state()
            state[3:6] = frd_to_flu(velocity_frd)
            state[10:13] = frd_to_flu(omega_frd)
            control = np.r_[
                float(row["collective_setpoint"]),
                max(0.0, float(row["pusher_actual"])),
                frd_to_flu(vector(row, "rate_sp")),
                float(row["lift_fraction_proxy"]),
            ]

            # Plant-only path: exact commands delivered by PX4/Gazebo bridge.
            target_rotor_speeds = np.asarray([
                float(row[f"motor_target_speed_{i}"]) for i in range(5)
            ])
            current_time = float(row["time_s"])
            dt = 0.02 if previous_time is None else np.clip(current_time - previous_time, 0.001, 0.1)
            previous_time = current_time
            for i, motor in enumerate(model.plant.motors):
                tau = (
                    motor.time_constant_up
                    if target_rotor_speeds[i] > logged_rotor_speeds[i]
                    else motor.time_constant_down
                )
                decay = np.exp(-dt / tau)
                logged_rotor_speeds[i] = (
                    decay * logged_rotor_speeds[i]
                    + (1.0 - decay) * target_rotor_speeds[i]
                )
            surface_commands = np.asarray([
                float(row[f"surface_angle_{i}"]) for i in range(3)
            ])
            logged_surface_state = model.step_surface_state(
                logged_surface_state, surface_commands, dt
            )
            velocity_flu = state[3:6]
            omega_flu = state[10:13]
            _, motor_torque = model.plant.motor_wrench(
                logged_rotor_speeds, velocity_flu, omega_flu
            )
            _, aero_torque = model.aerodynamic_wrench(
                logged_surface_state, velocity_flu, omega_flu,
                lift_fraction=float(row["lift_fraction_proxy"]),
            )
            logged_alpha_flu = np.linalg.solve(
                model.plant.inertia_b,
                motor_torque + aero_torque
                - np.cross(omega_flu, model.plant.inertia_b @ omega_flu),
            )

            # Complete future path. Stock PX4 keeps FW surfaces enabled through
            # transition, so override the future (1-lambda) schedule only for
            # this historical replay.
            fw_weight = 0.0 if zone == "mc" else 1.0
            _, predicted_surface_commands, _, _ = model.actuator_commands(
                state,
                control,
                angular_acceleration_flu=frd_to_flu(vector(row, "omega_dot")),
                mc_integrator_frd=vector(row, "mc_integrator"),
                fw_integrator_frd=vector(row, "fw_integrator"),
                fw_compression=vector(row, "fw_compression"),
                calibrated_airspeed=float(row["fw_filtered_airspeed_mps"]),
                fw_allocation_weight=fw_weight,
            )
            predicted_surface_state = model.step_surface_state(
                predicted_surface_state, predicted_surface_commands, dt
            )
            state[13:16] = predicted_surface_state
            predicted = model.derivative(
                state,
                control,
                angular_acceleration_flu=frd_to_flu(vector(row, "omega_dot")),
                mc_integrator_frd=vector(row, "mc_integrator"),
                fw_integrator_frd=vector(row, "fw_integrator"),
                fw_compression=vector(row, "fw_compression"),
                calibrated_airspeed=float(row["fw_filtered_airspeed_mps"]),
                fw_allocation_weight=fw_weight,
            )[10:13]

            measured = omega_dot_frd
            grouped[("logged_actuators", zone)]["measured"].append(measured)
            grouped[("logged_actuators", zone)]["logged"].append(flu_to_frd(logged_alpha_flu))
            grouped[("predicted_controller_allocator", zone)]["measured"].append(measured)
            grouped[("predicted_controller_allocator", zone)]["predicted"].append(flu_to_frd(predicted))

    results = {}
    for (path, zone), values in grouped.items():
        prediction_key = "logged" if path == "logged_actuators" else "predicted"
        results[(path, zone)] = metrics(values["measured"], values[prediction_key])

    options.output.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Standard VTOL rotational replay",
        "",
        "`logged_actuators` isolates the SDF rigid-body plant. `predicted_controller_allocator` uses rate setpoint, PX4 PID model, reconstructed allocator, identified surface-joint lag and SDF plant.",
        "",
        "| path | zone | samples | alpha RMSE p/q/r | correlation p/q/r | excited sign match p/q/r | measured std p/q/r | predicted std p/q/r | calibration gain p/q/r |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for path in ("logged_actuators", "predicted_controller_allocator"):
        for zone in ZONES:
            value = results[(path, zone)]
            lines.append(
                f"| {path} | {zone} | {value['samples']}"
                f" | {' / '.join(f'{v:.3f}' for v in value['rmse'])}"
                f" | {' / '.join(f'{v:.3f}' for v in value['correlation'])}"
                f" | {' / '.join(f'{v:.3f}' for v in value['excited_sign_match'])} |"
                f" {' / '.join(f'{v:.3f}' for v in value['measured_std'])} |"
                f" {' / '.join(f'{v:.3f}' for v in value['predicted_std'])} |"
                f" {' / '.join(f'{v:.3f}' for v in value['calibration_gain'])} |"
            )
    report = options.output / "rotational_replay_report.md"
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"report={report.resolve()}")
    for key, value in results.items():
        print(key, "rmse", value["rmse"], "corr", value["correlation"], "sign", value["excited_sign_match"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
