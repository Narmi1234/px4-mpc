#!/usr/bin/env python3
"""Fit a reduced body-force model using training and validation CSV files."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np
import yaml


PHASE_FIXED_WING = 4
AIR_DENSITY = 1.2041


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Fit the translational aerodynamic model on one run and evaluate "
            "it on a separate validation run."
        )
    )
    parser.add_argument("--train", type=Path, nargs="+", required=True)
    parser.add_argument("--validate", type=Path, nargs="+", required=True)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("results/standard_vtol_parameter_fit"),
    )
    parser.add_argument("--minimum-airspeed", type=float, default=10.0)
    parser.add_argument(
        "--ridge",
        type=float,
        default=1.0e-5,
        help="Small dimensionless regularization for poorly excited features",
    )
    return parser.parse_args()


def load_samples(path: Path, minimum_airspeed: float) -> dict[str, np.ndarray]:
    required = (
        "valid", "vtol_phase", "airspeed_mps", "left_elevon_rad",
        "right_elevon_rad", "elevator_rad", "velocity_body_x",
        "velocity_body_y", "velocity_body_z", "measured_aero_force_body_x",
        "measured_aero_force_body_y", "measured_aero_force_body_z",
        "sdf_aero_force_body_x", "sdf_aero_force_body_y",
        "sdf_aero_force_body_z",
    )
    selected = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        missing = set(required) - set(reader.fieldnames or ())
        if missing:
            names = ", ".join(sorted(missing))
            raise SystemExit(
                f"{path} lacks new force-identification columns: {names}. "
                "Re-run validate_standard_vtol_ulog.py first."
            )
        for row in reader:
            if int(row["valid"]) != 1 or int(row["vtol_phase"]) != PHASE_FIXED_WING:
                continue
            airspeed = float(row["airspeed_mps"])
            if not np.isfinite(airspeed) or airspeed < minimum_airspeed:
                continue
            selected.append(row)
    if len(selected) < 100:
        raise SystemExit(f"Only {len(selected)} usable fixed-wing samples in {path}")

    def column(name: str) -> np.ndarray:
        return np.asarray([float(row[name]) for row in selected])

    velocity = np.column_stack(
        [column(f"velocity_body_{axis}") for axis in ("x", "y", "z")]
    )
    measured = np.column_stack(
        [column(f"measured_aero_force_body_{axis}") for axis in ("x", "y", "z")]
    )
    sdf = np.column_stack(
        [column(f"sdf_aero_force_body_{axis}") for axis in ("x", "y", "z")]
    )
    return {
        "velocity": velocity,
        "measured": measured,
        "sdf": sdf,
        "airspeed": column("airspeed_mps"),
        "left": column("left_elevon_rad"),
        "right": column("right_elevon_rad"),
        "elevator": column("elevator_rad"),
    }


def load_many(paths: list[Path], minimum_airspeed: float) -> dict[str, np.ndarray]:
    runs = [load_samples(path, minimum_airspeed) for path in paths]
    return {
        name: np.concatenate([run[name] for run in runs], axis=0)
        for name in runs[0]
    }


def feature_matrices(data: dict[str, np.ndarray]) -> tuple[list[np.ndarray], list[list[str]]]:
    velocity = data["velocity"]
    forward = np.maximum(velocity[:, 0], 0.1)
    alpha = -np.arctan2(velocity[:, 2], forward)
    beta = np.arctan2(
        velocity[:, 1], np.sqrt(forward**2 + velocity[:, 2] ** 2)
    )
    dynamic_pressure = 0.5 * AIR_DENSITY * data["airspeed"] ** 2
    elevator = data["elevator"]

    features = [
        dynamic_pressure[:, None]
        * np.column_stack((np.ones(len(alpha)), alpha, alpha**2, elevator)),
        dynamic_pressure[:, None] * beta[:, None],
        dynamic_pressure[:, None]
        * np.column_stack((np.ones(len(alpha)), alpha, elevator)),
    ]
    names = [
        ["cx_0", "cx_alpha", "cx_alpha2", "cx_elevator"],
        ["cy_beta"],
        ["cz_0", "cz_alpha", "cz_elevator"],
    ]
    return features, names


def fit_axis(features: np.ndarray, target: np.ndarray, ridge: float) -> np.ndarray:
    scale = np.maximum(np.linalg.norm(features, axis=0), 1.0e-12)
    normalized = features / scale
    lhs = normalized.T @ normalized + ridge * np.eye(features.shape[1])
    normalized_coefficients = np.linalg.solve(lhs, normalized.T @ target)
    return normalized_coefficients / scale


def predict(data: dict[str, np.ndarray], coefficients: list[np.ndarray]) -> np.ndarray:
    features, _ = feature_matrices(data)
    return np.column_stack(
        [matrix @ axis_coefficients for matrix, axis_coefficients in zip(features, coefficients)]
    )


def metrics(measured: np.ndarray, predicted: np.ndarray) -> dict[str, np.ndarray]:
    residual = predicted - measured
    rmse = np.sqrt(np.mean(residual**2, axis=0))
    measured_rms = np.sqrt(np.mean(measured**2, axis=0))
    return {
        "rmse_force": rmse,
        "rmse_acceleration": rmse / 5.02500003,
        "nrmse_percent": 100.0 * rmse / np.maximum(measured_rms, 1.0e-12),
        "bias_force": np.mean(residual, axis=0),
    }


def serializable_metrics(values: dict[str, np.ndarray]) -> dict[str, list[float]]:
    return {name: array.tolist() for name, array in values.items()}


def main() -> int:
    options = arguments()
    train = load_many(options.train, options.minimum_airspeed)
    validation = load_many(options.validate, options.minimum_airspeed)
    train_features, names = feature_matrices(train)
    coefficients = [
        fit_axis(matrix, train["measured"][:, axis], options.ridge)
        for axis, matrix in enumerate(train_features)
    ]

    train_prediction = predict(train, coefficients)
    validation_prediction = predict(validation, coefficients)
    train_metrics = metrics(train["measured"], train_prediction)
    validation_metrics = metrics(validation["measured"], validation_prediction)
    validation_sdf_metrics = metrics(validation["measured"], validation["sdf"])

    parameter_map = {
        name: float(value)
        for axis_names, axis_values in zip(names, coefficients)
        for name, value in zip(axis_names, axis_values)
    }
    result = {
        "model": "standard_vtol_reduced_body_force_v1",
        "training_logs": [str(path) for path in options.train],
        "validation_logs": [str(path) for path in options.validate],
        "selection": {
            "phase": "fixed_wing",
            "minimum_airspeed_m_s": options.minimum_airspeed,
            "training_samples": len(train["airspeed"]),
            "validation_samples": len(validation["airspeed"]),
        },
        "equations": {
            "alpha": "-atan2(v_body_z, v_body_x)",
            "beta": "atan2(v_body_y, sqrt(v_body_x^2 + v_body_z^2))",
            "force": "F_body = qbar * Phi(alpha,beta,surfaces) * coefficients",
        },
        "parameters": parameter_map,
        "training_metrics": serializable_metrics(train_metrics),
        "validation_metrics": serializable_metrics(validation_metrics),
        "validation_sdf_metrics": serializable_metrics(validation_sdf_metrics),
    }

    options.output.mkdir(parents=True, exist_ok=True)
    parameters_path = options.output / "candidate_parameters.yaml"
    parameters_path.write_text(yaml.safe_dump(result, sort_keys=False), encoding="utf-8")

    axes = ("x", "y", "z")
    lines = [
        "# Reduced Standard VTOL aerodynamic fit",
        "",
        "- Training CSV: " + ", ".join(f"`{path}`" for path in options.train),
        "- Validation CSV: " + ", ".join(f"`{path}`" for path in options.validate),
        f"- Training samples: `{len(train['airspeed'])}`",
        f"- Validation samples: `{len(validation['airspeed'])}`",
        "- Candidate parameters are not promoted to the NMPC model automatically.",
        "",
        "## Validation comparison",
        "",
        "| Model | accel RMSE x | accel RMSE y | accel RMSE z |",
        "|---|---:|---:|---:|",
        "| SDF baseline | "
        + " | ".join(f"{v:.3f}" for v in validation_sdf_metrics["rmse_acceleration"])
        + " |",
        "| identified candidate | "
        + " | ".join(f"{v:.3f}" for v in validation_metrics["rmse_acceleration"])
        + " |",
        "",
        "## Coefficients",
        "",
        "| Parameter | Value |",
        "|---|---:|",
    ]
    for name, value in parameter_map.items():
        lines.append(f"| `{name}` | {value:.8g} |")
    lines.extend(
        [
            "",
            "## Decision gate",
            "",
            "Promote this candidate only if it improves the untouched validation run "
            "on the required axes, coefficient signs are physically plausible, and "
            "the regression is repeated with a third flight containing different "
            "airspeed and pitch/elevator excitation. Moments are intentionally not "
            "identified: the first NMPC uses PX4's closed body-rate loop.",
        ]
    )
    (options.output / "fit_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    print("Validation acceleration RMSE SDF [x,y,z]:", validation_sdf_metrics["rmse_acceleration"])
    print("Validation acceleration RMSE fit [x,y,z]:", validation_metrics["rmse_acceleration"])
    print(f"Report: {(options.output / 'fit_report.md').resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
