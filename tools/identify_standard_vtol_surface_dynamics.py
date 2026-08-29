#!/usr/bin/env python3
"""Identify the unlogged Gazebo control-surface joint response from ULogs.

The Standard VTOL SDF sends PX4 servo commands to a JointPositionController.
LiftDrag uses the *joint position*, while PX4 only logs the commanded angle.
This tool fits a first-order effective joint model on one dataset and evaluates
the frozen parameters on another dataset.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "px4_mpc"))

from px4_mpc.models.frames import frd_to_flu, flu_to_frd  # noqa: E402
from px4_mpc.models.standard_vtol_rotational_model import (  # noqa: E402
    StandardVtolTorqueInformedModel,
)


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train", type=Path, required=True)
    parser.add_argument("--validate", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def _vector(row, prefix, axes):
    return np.asarray([float(row[f"{prefix}_{axis}"]) for axis in axes])


def load_features(path: Path, model: StandardVtolTorqueInformedModel):
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))

    count = len(rows)
    time = np.asarray([float(row["time_s"]) for row in rows])
    command = np.asarray([
        [float(row[f"surface_angle_{index}"]) for index in range(3)]
        for row in rows
    ])
    measured = np.asarray([_vector(row, "omega_dot", ("p", "q", "r")) for row in rows])
    valid = np.asarray([
        int(row["valid"]) == 1 and row["zone"] in ("blend", "fw")
        for row in rows
    ])

    baseline = np.zeros((count, 3))
    effectiveness = np.zeros((count, 3, 3))
    rotor_speed = np.zeros(5)
    previous_time = None
    for index, row in enumerate(rows):
        dt = 0.02 if previous_time is None else float(np.clip(time[index] - previous_time, 0.001, 0.1))
        previous_time = time[index]
        targets = np.asarray([
            float(row[f"motor_target_speed_{motor}"]) for motor in range(5)
        ])
        for motor_index, motor in enumerate(model.plant.motors):
            tau = motor.time_constant_up if targets[motor_index] > rotor_speed[motor_index] else motor.time_constant_down
            decay = np.exp(-dt / tau)
            rotor_speed[motor_index] = decay * rotor_speed[motor_index] + (1.0 - decay) * targets[motor_index]

        velocity = frd_to_flu(_vector(row, "velocity_body", ("x", "y", "z")))
        omega = frd_to_flu(_vector(row, "omega", ("p", "q", "r")))
        _, motor_torque = model.plant.motor_wrench(rotor_speed, velocity, omega)
        _, neutral_torque = model.plant.aerodynamic_wrench(np.zeros(3), velocity, omega)
        gyroscopic = np.cross(omega, model.plant.inertia_b @ omega)
        alpha_neutral = np.linalg.solve(
            model.plant.inertia_b, motor_torque + neutral_torque - gyroscopic
        )
        baseline[index] = flu_to_frd(alpha_neutral)

        for surface in range(3):
            unit = np.zeros(3)
            unit[surface] = 1.0
            _, unit_torque = model.plant.aerodynamic_wrench(unit, velocity, omega)
            delta_alpha = np.linalg.solve(
                model.plant.inertia_b, unit_torque - neutral_torque
            )
            effectiveness[index, :, surface] = flu_to_frd(delta_alpha)

    return {
        "time": time,
        "command": command,
        "measured": measured,
        "valid": valid,
        "baseline": baseline,
        "effectiveness": effectiveness,
    }


def filtered_surfaces(features, time_constants, gains):
    command = features["command"]
    time = features["time"]
    actual = np.zeros_like(command)
    for index in range(1, len(command)):
        dt = float(np.clip(time[index] - time[index - 1], 0.001, 0.1))
        decay = np.exp(-dt / time_constants)
        actual[index] = decay * actual[index - 1] + (1.0 - decay) * gains * command[index]
    return actual


def prediction(features, time_constants, gains):
    actual = filtered_surfaces(features, time_constants, gains)
    alpha = features["baseline"] + np.einsum(
        "nij,nj->ni", features["effectiveness"], actual
    )
    return actual, alpha


def axis_metrics(features, alpha):
    mask = features["valid"] & np.all(np.isfinite(alpha), axis=1)
    measured = features["measured"][mask]
    predicted = alpha[mask]
    residual = predicted - measured
    correlation = []
    for axis in range(3):
        if np.std(measured[:, axis]) < 1e-12 or np.std(predicted[:, axis]) < 1e-12:
            correlation.append(0.0)
        else:
            correlation.append(float(np.corrcoef(measured[:, axis], predicted[:, axis])[0, 1]))
    return {
        "samples": int(np.sum(mask)),
        "rmse": np.sqrt(np.mean(residual**2, axis=0)),
        "correlation": np.asarray(correlation),
    }


def fit(train):
    mask = train["valid"]
    scale = np.maximum(np.std(train["measured"][mask, :2], axis=0), 0.1)

    def residual(parameters):
        compact = np.exp(parameters)
        time_constants = np.asarray([compact[0], compact[0], compact[1]])
        gains = np.asarray([compact[2], compact[2], compact[3]])
        _, alpha = prediction(train, time_constants, gains)
        # Elevons/elevator identify roll and pitch. Yaw is intentionally left
        # out because none of the three aerodynamic surfaces commands yaw.
        return ((alpha[mask, :2] - train["measured"][mask, :2]) / scale).ravel()

    # The left and right elevons have identical SDF joint/controller dynamics,
    # so constrain them to the same time constant and static gain. This avoids
    # using flight asymmetry to invent two physically different servos.
    initial = np.log(np.ones(4))
    lower = np.log(np.r_[np.full(2, 0.01), np.full(2, 0.1)])
    upper = np.log(np.r_[np.full(2, 3.0), np.full(2, 2.0)])
    result = least_squares(
        residual, initial, bounds=(lower, upper), loss="soft_l1", f_scale=0.5,
        max_nfev=120,
    )
    compact = np.exp(result.x)
    return (
        np.asarray([compact[0], compact[0], compact[1]]),
        np.asarray([compact[2], compact[2], compact[3]]),
        result,
    )


def fmt(values):
    return " / ".join(f"{value:.4f}" for value in values)


def main():
    options = arguments()
    model = StandardVtolTorqueInformedModel()
    train = load_features(options.train, model)
    validate = load_features(options.validate, model)
    time_constants, gains, optimizer = fit(train)

    cases = {}
    for name, features in (("train", train), ("validation", validate)):
        _, instantaneous = prediction(features, np.full(3, 0.01), np.ones(3))
        _, sdf_nominal = prediction(features, np.ones(3), np.ones(3))
        _, lagged = prediction(features, time_constants, gains)
        cases[(name, "instantaneous_command")] = axis_metrics(features, instantaneous)
        cases[(name, "sdf_nominal_joint")] = axis_metrics(features, sdf_nominal)
        cases[(name, "identified_joint")] = axis_metrics(features, lagged)

    options.output.mkdir(parents=True, exist_ok=True)
    parameters = {
        "surface_time_constants_s": time_constants.tolist(),
        "surface_static_gains": gains.tolist(),
        "training_dataset": str(options.train),
        "validation_dataset": str(options.validate),
        "optimizer_success": bool(optimizer.success),
        "optimizer_cost": float(optimizer.cost),
    }
    (options.output / "surface_dynamics.json").write_text(
        json.dumps(parameters, indent=2) + "\n", encoding="utf-8"
    )
    lines = [
        "# Standard VTOL effective surface dynamics",
        "",
        "The fit uses only the training ULog. Frozen parameters are then evaluated on the validation ULog.",
        "",
        f"- time constants left/right/elevator: `{fmt(time_constants)} s`",
        f"- static gains left/right/elevator: `{fmt(gains)}`",
        "",
        "| dataset | model | samples | alpha RMSE p/q/r | correlation p/q/r |",
        "|---|---|---:|---:|---:|",
    ]
    for dataset in ("train", "validation"):
        for case in ("instantaneous_command", "sdf_nominal_joint", "identified_joint"):
            value = cases[(dataset, case)]
            lines.append(
                f"| {dataset} | {case} | {value['samples']} | {fmt(value['rmse'])} | {fmt(value['correlation'])} |"
            )
    (options.output / "surface_dynamics_report.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )
    print(json.dumps(parameters, indent=2))
    for key, value in cases.items():
        print(key, "rmse", value["rmse"], "corr", value["correlation"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
