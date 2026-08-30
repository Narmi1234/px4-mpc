#!/usr/bin/env python3
"""Fit stable blend pitch coefficients on 0.5 s training rollouts."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "px4_mpc"))

from px4_mpc.models.standard_vtol_rotational_model import (  # noqa: E402
    StandardVtolTorqueInformedModel,
)
from tools.validate_standard_vtol_rotational_rollout import (  # noqa: E402
    reconstruct_initial_actuators,
    rollout,
    vector,
)


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train", type=Path, required=True)
    parser.add_argument("--validate", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--horizon", type=float, default=0.5)
    parser.add_argument("--stride", type=int, default=5)
    parser.add_argument("--regularization", type=float, default=0.1)
    return parser.parse_args()


def load(path):
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def blend_windows(rows, horizon, stride):
    dt = float(np.median(np.diff([float(row["time_s"]) for row in rows])))
    steps = max(1, int(round(horizon / dt)))
    starts = []
    for start in range(0, len(rows) - steps, stride):
        window = rows[start:start + steps + 1]
        if all(int(row["valid"]) == 1 and row["zone"] == "blend" for row in window):
            starts.append(start)
    return starts, steps


def endpoint_errors(rows, surfaces, starts, steps, model):
    return np.asarray([
        rollout(rows, surfaces, start, steps, model)[1]
        - vector(rows[start + steps], "omega")[1]
        for start in starts
    ])


def main():
    options = arguments()
    model = StandardVtolTorqueInformedModel()
    train_rows = load(options.train)
    train_surfaces, _ = reconstruct_initial_actuators(train_rows, model)
    train_starts, steps = blend_windows(
        train_rows, options.horizon, options.stride
    )
    if not train_starts:
        raise RuntimeError("training dataset has no complete blend rollout windows")

    prior = model.pitch_moment_coefficients_blend.copy()
    scale = np.asarray([0.01, 0.2, 1.0, 0.1, 0.05, 1.0])
    initial = prior.copy()
    initial[3] = min(initial[3], -1.0e-4)
    lower = np.asarray([-0.1, -2.0, -5.0, -2.0, -0.5, -3.0])
    upper = np.asarray([0.1, 2.0, 5.0, 0.0, 0.5, 1.0])

    def residual(coefficients):
        model.pitch_moment_coefficients_blend = coefficients
        flight = endpoint_errors(
            train_rows, train_surfaces, train_starts, steps, model
        ) / 0.05
        regularization = np.sqrt(options.regularization) * (
            coefficients - prior
        ) / scale
        return np.r_[flight, regularization]

    fit = least_squares(
        residual,
        initial,
        bounds=(lower, upper),
        loss="soft_l1",
        f_scale=1.0,
        max_nfev=80,
        verbose=1,
    )
    coefficients = fit.x
    model.pitch_moment_coefficients_blend = coefficients

    validate_rows = load(options.validate)
    validate_surfaces, _ = reconstruct_initial_actuators(validate_rows, model)
    validate_starts, validate_steps = blend_windows(
        validate_rows, options.horizon, options.stride
    )
    train_error = endpoint_errors(
        train_rows, train_surfaces, train_starts, steps, model
    )
    validate_error = endpoint_errors(
        validate_rows, validate_surfaces, validate_starts, validate_steps, model
    )
    result = {
        "coefficients": coefficients.tolist(),
        "prior_coefficients": prior.tolist(),
        "pitch_damping_constraint": "coefficient[3] <= 0",
        "horizon_s": options.horizon,
        "regularization": options.regularization,
        "training_windows": len(train_starts),
        "validation_windows": len(validate_starts),
        "training_pitch_rmse_rad_s": float(np.sqrt(np.mean(train_error**2))),
        "validation_pitch_rmse_rad_s": float(np.sqrt(np.mean(validate_error**2))),
        "optimizer_success": bool(fit.success),
        "optimizer_message": fit.message,
    }
    options.output.mkdir(parents=True, exist_ok=True)
    (options.output / "pitch_rollout.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8"
    )
    report = [
        "# Stable blend pitch rollout fit",
        "",
        "Only the training ULog is used by the optimizer. Validation is evaluated after coefficients are frozen.",
        "",
        f"- coefficients: `{coefficients.tolist()}`",
        f"- training windows/RMSE: `{len(train_starts)} / {result['training_pitch_rmse_rad_s']:.5f} rad/s`",
        f"- validation windows/RMSE: `{len(validate_starts)} / {result['validation_pitch_rmse_rad_s']:.5f} rad/s`",
        f"- damping constraint: `{result['pitch_damping_constraint']}`",
        f"- optimizer: `{fit.message}`",
    ]
    (options.output / "pitch_rollout_report.md").write_text(
        "\n".join(report) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
