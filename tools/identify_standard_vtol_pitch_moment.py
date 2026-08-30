#!/usr/bin/env python3
"""Identify an effective Standard VTOL pitch-moment model from ULogs."""

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


def vector(row, prefix, axes):
    return np.asarray([float(row[f"{prefix}_{axis}"]) for axis in axes])


def load(path: Path, model: StandardVtolTorqueInformedModel):
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    rotor_speed = np.zeros(5)
    surface_state = np.zeros(3)
    previous_time = None
    features = []
    required_moment = []
    sdf_prediction = []
    zone = []
    for row in rows:
        time = float(row["time_s"])
        dt = 0.02 if previous_time is None else float(np.clip(time - previous_time, 0.001, 0.1))
        previous_time = time
        targets = np.asarray([float(row[f"motor_target_speed_{i}"]) for i in range(5)])
        for i, motor in enumerate(model.plant.motors):
            tau = motor.time_constant_up if targets[i] > rotor_speed[i] else motor.time_constant_down
            decay = np.exp(-dt / tau)
            rotor_speed[i] = decay * rotor_speed[i] + (1.0 - decay) * targets[i]
        surface_command = np.asarray([float(row[f"surface_angle_{i}"]) for i in range(3)])
        surface_state = model.step_surface_state(surface_state, surface_command, dt)

        velocity = frd_to_flu(vector(row, "velocity_body", ("x", "y", "z")))
        omega = frd_to_flu(vector(row, "omega", ("p", "q", "r")))
        omega_dot = frd_to_flu(vector(row, "omega_dot", ("p", "q", "r")))
        forward_speed = max(float(velocity[0]), 0.0)
        alpha = float(np.arctan2(-velocity[2], max(forward_speed, 0.1)))
        dynamic_pressure = 0.5 * model.plant.aero_surfaces[0].air_density * forward_speed * forward_speed
        reduced_pitch_rate = omega[1] / max(forward_speed, 1.0)
        # Effective dimensional model. Coefficients multiply dynamic pressure;
        # areas and reference chord are absorbed into the fitted parameters.
        phi = dynamic_pressure * np.asarray([
            1.0,
            alpha,
            alpha * abs(alpha),
            reduced_pitch_rate,
            surface_state[2],
        ])

        _, motor_torque = model.plant.motor_wrench(rotor_speed, velocity, omega)
        _, aero_torque = model.plant.aerodynamic_wrench(surface_state, velocity, omega)
        rigid_body_torque = model.plant.inertia_b @ omega_dot + np.cross(
            omega, model.plant.inertia_b @ omega
        )
        features.append(np.r_[phi, motor_torque[1]])
        required_moment.append(rigid_body_torque[1] - motor_torque[1])
        sdf_prediction.append(aero_torque[1])
        zone.append(row["zone"] if int(row["valid"]) == 1 else "invalid")
    return {
        "features": np.asarray(features),
        "required": np.asarray(required_moment),
        "sdf": np.asarray(sdf_prediction),
        "zone": np.asarray(zone),
    }


def fit(train):
    coefficients = {}
    optimizers = {}
    for selected_zone in ("blend", "fw"):
        mask = (
            np.isin(train["zone"], ("blend", "fw"))
            if selected_zone == "blend"
            else train["zone"] == "fw"
        )
        x = train["features"][mask]
        y = train["required"][mask]
        weights = np.ones(len(y))
        if selected_zone == "blend":
            zones = train["zone"][mask]
            counts = {zone: max(1, int(np.sum(zones == zone))) for zone in ("blend", "fw")}
            weights = np.asarray([1.0 / np.sqrt(counts[zone]) for zone in zones])
            weights /= np.mean(weights)
        scale = np.maximum(np.std(x, axis=0), 1e-6)

        def residual(scaled_coefficients):
            return weights * ((x / scale) @ scaled_coefficients - y)

        lower = np.full(x.shape[1], -np.inf)
        upper = np.full(x.shape[1], np.inf)
        # Feature 3 is qbar*q/V. Positive aerodynamic pitch damping is
        # nonphysical and made earlier multi-step rollouts diverge.
        upper[3] = 0.0
        initial = np.zeros(x.shape[1])
        initial[3] = -1.0e-3 * scale[3]
        result = least_squares(
            residual, initial, bounds=(lower, upper),
            loss="soft_l1", f_scale=0.1,
            max_nfev=100,
        )
        coefficients[selected_zone] = result.x / scale
        optimizers[selected_zone] = result
    return coefficients, optimizers


def metrics(data, coefficients, selected_zone):
    mask = data["zone"] == selected_zone
    measured = data["required"][mask]
    candidates = {
        "sdf": data["sdf"][mask],
        "identified": data["features"][mask] @ coefficients[selected_zone],
    }
    result = {}
    for name, predicted in candidates.items():
        residual = predicted - measured
        result[name] = {
            "samples": int(np.sum(mask)),
            "rmse_nm": float(np.sqrt(np.mean(residual**2))),
            "correlation": float(np.corrcoef(measured, predicted)[0, 1]),
            "measured_std_nm": float(np.std(measured)),
            "predicted_std_nm": float(np.std(predicted)),
        }
    return result


def main():
    options = arguments()
    model = StandardVtolTorqueInformedModel()
    train = load(options.train, model)
    validate = load(options.validate, model)
    coefficients, optimizers = fit(train)
    names = [
        "qbar", "qbar_alpha", "qbar_alpha_abs", "qbar_q_over_v",
        "qbar_elevator", "motor_pitch_residual",
    ]
    result = {
        "feature_names": names,
        "coefficients": {
            "blend_balanced": coefficients["blend"].tolist(),
            "fw": coefficients["fw"].tolist(),
        },
        "training_dataset": str(options.train),
        "validation_dataset": str(options.validate),
        "optimizer_success": {zone: bool(result.success) for zone, result in optimizers.items()},
    }
    all_metrics = {}
    for dataset_name, data in (("train", train), ("validation", validate)):
        for selected_zone in ("blend", "fw"):
            all_metrics[(dataset_name, selected_zone)] = metrics(
                data, coefficients, selected_zone
            )

    options.output.mkdir(parents=True, exist_ok=True)
    (options.output / "pitch_moment.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8"
    )
    lines = [
        "# Standard VTOL effective pitch moment",
        "",
        "Coefficients were fitted only on the training ULog and frozen before validation.",
        "",
        "```text",
        "M_y = qbar * (c0 + c_alpha*alpha + c_alpha2*alpha*abs(alpha)",
        "                + c_q*q/V + c_de*delta_elevator)",
        "      + c_motor*M_motor,y",
        "```",
        "",
        "| feature | coefficient |",
        "|---|---:|",
    ]
    for name, blend_value, fw_value in zip(names, coefficients["blend"], coefficients["fw"]):
        lines.append(f"| {name} | {blend_value:.8g} (blend), {fw_value:.8g} (FW) |")
    lines.extend([
        "",
        "| dataset | zone | model | samples | moment RMSE [Nm] | correlation | measured std | predicted std |",
        "|---|---|---|---:|---:|---:|---:|---:|",
    ])
    for dataset_name in ("train", "validation"):
        for selected_zone in ("blend", "fw"):
            for candidate, value in all_metrics[(dataset_name, selected_zone)].items():
                lines.append(
                    f"| {dataset_name} | {selected_zone} | {candidate} | {value['samples']} | "
                    f"{value['rmse_nm']:.4f} | {value['correlation']:.4f} | "
                    f"{value['measured_std_nm']:.4f} | {value['predicted_std_nm']:.4f} |"
                )
    (options.output / "pitch_moment_report.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, indent=2))
    for key, value in all_metrics.items():
        print(key, value)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
