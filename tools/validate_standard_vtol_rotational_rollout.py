#!/usr/bin/env python3
"""Validate 0.5 s body-rate rollouts without future torque/servo inputs."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "px4_mpc"))

from px4_mpc.models.frames import frd_to_flu, flu_to_frd  # noqa: E402
from px4_mpc.models.standard_vtol_rotational_model import (  # noqa: E402
    StandardVtolTorqueInformedModel,
)


AXES = ("p", "q", "r")
ZONES = ("mc", "blend", "fw")


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, nargs="+")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--horizon", type=float, default=0.5)
    parser.add_argument("--stride", type=int, default=5)
    parser.add_argument("--zones", nargs="+", choices=ZONES, default=list(ZONES))
    return parser.parse_args()


def vector(row, prefix, axes=AXES):
    return np.asarray([float(row[f"{prefix}_{axis}"]) for axis in axes])


def reconstruct_initial_actuators(rows, model):
    surfaces = np.zeros((len(rows), 3))
    rotors = np.zeros((len(rows), 5))
    previous_time = None
    state = np.zeros(3)
    rotor_state = np.zeros(5)
    for index, row in enumerate(rows):
        time = float(row["time_s"])
        dt = 0.02 if previous_time is None else float(np.clip(time - previous_time, 0.001, 0.1))
        previous_time = time
        command = np.asarray([float(row[f"surface_angle_{i}"]) for i in range(3)])
        state = model.step_surface_state(state, command, dt)
        targets = np.asarray([float(row[f"motor_target_speed_{i}"]) for i in range(5)])
        for motor_index, motor in enumerate(model.plant.motors):
            tau = motor.time_constant_up if targets[motor_index] > rotor_state[motor_index] else motor.time_constant_down
            decay = np.exp(-dt / tau)
            rotor_state[motor_index] = decay * rotor_state[motor_index] + (1.0 - decay) * targets[motor_index]
        surfaces[index] = state
        rotors[index] = rotor_state
    return surfaces, rotors


def rollout(rows, initial_surfaces, start, steps, model):
    first = rows[start]
    state = model.hover_state()
    state[10:13] = frd_to_flu(vector(first, "omega"))
    state[13:16] = initial_surfaces[start]
    previous_alpha = frd_to_flu(vector(first, "omega_dot"))
    mc_integrator = vector(first, "mc_integrator")
    fw_integrator = vector(first, "fw_integrator")
    fw_compression = vector(first, "fw_compression")

    for offset in range(steps):
        row = rows[start + offset]
        next_row = rows[start + offset + 1]
        dt = float(np.clip(float(next_row["time_s"]) - float(row["time_s"]), 0.001, 0.1))
        state[3:6] = frd_to_flu(vector(row, "velocity_body", ("x", "y", "z")))
        control = np.r_[
            float(row["collective_setpoint"]),
            max(0.0, float(row["pusher_actual"])),
            frd_to_flu(vector(row, "rate_sp")),
            float(row["lift_fraction_proxy"]),
        ]
        fw_weight = 0.0 if row["zone"] == "mc" else 1.0
        parameters = {
            "angular_acceleration_flu": previous_alpha,
            "mc_integrator_frd": mc_integrator,
            "fw_integrator_frd": fw_integrator,
            "fw_compression": fw_compression,
            "calibrated_airspeed": float(row["fw_filtered_airspeed_mps"]),
            "fw_allocation_weight": fw_weight,
        }
        derivative = model.derivative(state, control, **parameters)
        _, surface_command, _, _ = model.actuator_commands(state, control, **parameters)
        state[10:13] += dt * derivative[10:13]
        state[13:16] = model.step_surface_state(
            state[13:16], surface_command, dt
        )
        previous_alpha = derivative[10:13]
    return flu_to_frd(state[10:13])


def rollout_logged_actuators(rows, initial_surfaces, initial_rotors, start, steps, model):
    state = model.hover_state()
    state[10:13] = frd_to_flu(vector(rows[start], "omega"))
    surface_state = initial_surfaces[start].copy()
    rotor_state = initial_rotors[start].copy()
    for offset in range(steps):
        row = rows[start + offset]
        next_row = rows[start + offset + 1]
        dt = float(np.clip(float(next_row["time_s"]) - float(row["time_s"]), 0.001, 0.1))
        velocity = frd_to_flu(vector(row, "velocity_body", ("x", "y", "z")))
        omega = state[10:13]
        _, motor_torque = model.plant.motor_wrench(rotor_state, velocity, omega)
        _, aero_torque = model.aerodynamic_wrench(
            surface_state, velocity, omega,
            lift_fraction=float(row["lift_fraction_proxy"]),
        )
        alpha = np.linalg.solve(
            model.plant.inertia_b,
            motor_torque + aero_torque
            - np.cross(omega, model.plant.inertia_b @ omega),
        )
        state[10:13] += dt * alpha

        surface_command = np.asarray([float(row[f"surface_angle_{i}"]) for i in range(3)])
        surface_state = model.step_surface_state(surface_state, surface_command, dt)
        targets = np.asarray([float(row[f"motor_target_speed_{i}"]) for i in range(5)])
        for motor_index, motor in enumerate(model.plant.motors):
            tau = motor.time_constant_up if targets[motor_index] > rotor_state[motor_index] else motor.time_constant_down
            decay = np.exp(-dt / tau)
            rotor_state[motor_index] = decay * rotor_state[motor_index] + (1.0 - decay) * targets[motor_index]
    return flu_to_frd(state[10:13])


def metric(errors):
    errors = np.asarray(errors)
    return np.sqrt(np.mean(errors**2, axis=0)) if len(errors) else np.full(3, np.nan)


def main():
    options = arguments()
    model = StandardVtolTorqueInformedModel()
    selected_zones = tuple(options.zones)
    errors = {zone: [] for zone in selected_zones}
    baselines = {zone: [] for zone in selected_zones}
    plant_errors = {zone: [] for zone in selected_zones}
    counts = {zone: 0 for zone in selected_zones}
    for path in options.csv:
        with path.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
        initial_surfaces, initial_rotors = reconstruct_initial_actuators(rows, model)
        nominal_dt = float(np.median(np.diff([float(row["time_s"]) for row in rows])))
        steps = max(1, int(round(options.horizon / nominal_dt)))
        for start in range(0, len(rows) - steps, options.stride):
            window = rows[start:start + steps + 1]
            zone = rows[start]["zone"]
            if zone not in selected_zones:
                continue
            if not all(int(row["valid"]) == 1 and row["zone"] == zone for row in window):
                continue
            predicted = rollout(rows, initial_surfaces, start, steps, model)
            plant_predicted = rollout_logged_actuators(
                rows, initial_surfaces, initial_rotors, start, steps, model
            )
            measured_initial = vector(rows[start], "omega")
            measured_final = vector(rows[start + steps], "omega")
            errors[zone].append(predicted - measured_final)
            plant_errors[zone].append(plant_predicted - measured_final)
            baselines[zone].append(measured_initial - measured_final)
            counts[zone] += 1

    options.output.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Standard VTOL 0.5 s rotational rollout",
        "",
        "Future logged torque and servo commands are not used. PID integrator and gain-compression states are frozen at each window start; rate, collective and pusher references remain known inputs.",
        "",
        "`logged-actuator plant` is diagnostic only and deliberately uses future logged actuator inputs. The acceptance model does not.",
        "",
        "| zone | windows | rate-sp model RMSE p/q/r | logged-actuator plant RMSE p/q/r | zero-order-hold RMSE p/q/r | gate |",
        "|---|---:|---:|---:|---:|---|",
    ]
    overall_pass = True
    for zone in selected_zones:
        value = metric(errors[zone])
        plant_value = metric(plant_errors[zone])
        baseline = metric(baselines[zone])
        threshold = 0.05 if zone in ("blend", "fw") else 0.03
        passed = bool(np.all(np.isfinite(value)) and value[1] <= threshold)
        overall_pass &= passed
        lines.append(
            f"| {zone} | {counts[zone]} | {' / '.join(f'{v:.4f}' for v in value)} | "
            f"{' / '.join(f'{v:.4f}' for v in plant_value)} | "
            f"{' / '.join(f'{v:.4f}' for v in baseline)} | {'PASS' if passed else 'FAIL'} |"
        )
    lines.extend(["", f"ROTATIONAL_ROLLOUT_GATE={'PASS' if overall_pass else 'FAIL'}"])
    report = options.output / "rotational_rollout_report.md"
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(report.read_text(encoding="utf-8"))
    return 0 if overall_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
