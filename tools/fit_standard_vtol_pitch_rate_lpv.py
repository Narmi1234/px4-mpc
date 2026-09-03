#!/usr/bin/env python3
"""Fit a stable Markov LPV pitch-rate model and quantify its uncertainty."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares


ZONES = ("mc", "blend", "fw")


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train", type=Path, nargs="+", required=True)
    parser.add_argument("--validate", type=Path, nargs="+", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--horizon", type=float, default=0.5)
    parser.add_argument("--stride", type=int, default=5)
    parser.add_argument("--minimum-damping", type=float, default=0.02)
    parser.add_argument(
        "--required-zones", nargs="+", choices=ZONES,
        default=["blend", "fw"],
        help="zones that must satisfy the validation gate",
    )
    return parser.parse_args()


def load(path):
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    required = {
        "valid", "zone", "airspeed_mps", "lift_fraction_proxy",
        "alpha_proxy_rad", "pitch_rad", "rate_sp_q", "omega_q", "omega_dot_q",
    }
    missing = required - set(rows[0] if rows else ())
    if missing:
        raise RuntimeError(f"{path} missing columns: {', '.join(sorted(missing))}")
    airspeed = np.clip([float(row["airspeed_mps"]) for row in rows], 0.0, 25.0)
    lift_fraction = np.clip(
        [float(row["lift_fraction_proxy"]) for row in rows], 0.0, 1.0
    )
    mu = 1.0 - lift_fraction
    speed = np.asarray(airspeed) / 20.0
    basis = np.column_stack([
        (1.0 - speed) * (1.0 - mu),
        speed * (1.0 - mu),
        (1.0 - speed) * mu,
        speed * mu,
    ])
    transition = 4.0 * mu * (1.0 - mu)
    return {
        "path": str(path),
        "valid": np.asarray([int(row["valid"]) == 1 for row in rows]),
        "zone": np.asarray([row["zone"] for row in rows]),
        "q": np.asarray([float(row["omega_q"]) for row in rows]),
        "q_dot": np.asarray([float(row["omega_dot_q"]) for row in rows]),
        "q_sp": np.asarray([float(row["rate_sp_q"]) for row in rows]),
        "basis": basis,
        "transition": transition,
        "alpha": np.clip(
            [float(row["alpha_proxy_rad"]) for row in rows], -0.7, 0.7
        ),
        "pitch": np.clip([float(row["pitch_rad"]) for row in rows], -0.7, 0.7),
    }


def feature(run, index, q=None):
    q_value = run["q"][index] if q is None else float(q)
    basis = run["basis"][index]
    transition = run["transition"][index]
    return np.r_[
        -basis * q_value,
        basis * run["q_sp"][index],
        transition,
        transition * run["alpha"][index],
        transition * run["pitch"][index],
    ]


FEATURE_NAMES = [
    "minus_q_fw_low", "minus_q_fw_high", "minus_q_mc_low", "minus_q_mc_high",
    "q_sp_fw_low", "q_sp_fw_high", "q_sp_mc_low", "q_sp_mc_high",
    "transition_bias", "transition_alpha", "transition_pitch",
]


def training_samples(runs):
    x_blocks, y_blocks, weight_blocks = [], [], []
    for run in runs:
        for zone in ZONES:
            mask = run["valid"] & (run["zone"] == zone)
            count = int(np.count_nonzero(mask))
            if not count:
                continue
            x_blocks.append(np.asarray([
                feature(run, index) for index in np.flatnonzero(mask)
            ]))
            y_blocks.append(run["q_dot"][mask])
            # Each flight/zone pair has equal total least-squares energy.
            weight_blocks.append(np.full(count, 1.0 / np.sqrt(count)))
    x = np.vstack(x_blocks)
    y = np.concatenate(y_blocks)
    weights = np.concatenate(weight_blocks)
    weights /= np.mean(weights)
    return x, y, weights


def fit(runs, minimum_damping):
    x, y, weights = training_samples(runs)
    scale = np.maximum(np.std(x, axis=0), 1.0e-6)
    lower = np.r_[
        np.full(4, minimum_damping), np.zeros(4), np.full(3, -np.inf)
    ]
    upper = np.full(len(FEATURE_NAMES), np.inf)
    initial = np.r_[np.ones(4), np.ones(4), np.zeros(3)] * scale
    result = least_squares(
        lambda normalized: weights * ((x / scale) @ normalized - y),
        initial,
        bounds=(lower * scale, upper * scale),
        loss="soft_l1",
        f_scale=0.2,
        max_nfev=200,
    )
    return result.x / scale, result


def rollout_errors(run, coefficients, horizon, stride):
    errors = {zone: [] for zone in ZONES}
    for start in range(0, len(run["q"]) - horizon, stride):
        zone = run["zone"][start]
        if zone not in errors:
            continue
        window = slice(start, start + horizon + 1)
        if not np.all(run["valid"][window]):
            continue
        if not np.all(run["zone"][window] == zone):
            continue
        predicted = float(run["q"][start])
        for index in range(start, start + horizon):
            predicted += 0.02 * float(feature(run, index, predicted) @ coefficients)
        errors[zone].append(predicted - run["q"][start + horizon])
    return errors


def metrics(runs, coefficients, horizon, stride):
    pointwise = {zone: [] for zone in ZONES}
    endpoint = {zone: [] for zone in ZONES}
    per_run = {}
    for run in runs:
        predicted_dot = np.asarray([
            feature(run, index) @ coefficients for index in range(len(run["q"]))
        ])
        for zone in ZONES:
            mask = run["valid"] & (run["zone"] == zone)
            pointwise[zone].extend((predicted_dot[mask] - run["q_dot"][mask]).tolist())
        errors = rollout_errors(run, coefficients, horizon, stride)
        per_run[run["path"]] = {}
        for zone in ZONES:
            endpoint[zone].extend(errors[zone])
            values = np.asarray(errors[zone])
            per_run[run["path"]][zone] = {
                "windows": len(values),
                "endpoint_rmse_rad_s": float(np.sqrt(np.mean(values**2))) if len(values) else None,
                "endpoint_p95_abs_rad_s": float(np.percentile(np.abs(values), 95)) if len(values) else None,
            }
    aggregate = {}
    for zone in ZONES:
        derivative = np.asarray(pointwise[zone])
        errors = np.asarray(endpoint[zone])
        aggregate[zone] = {
            "samples": len(derivative),
            "q_dot_rmse_rad_s2": (
                float(np.sqrt(np.mean(derivative**2))) if len(derivative) else None
            ),
            "q_dot_p95_abs_rad_s2": (
                float(np.percentile(np.abs(derivative), 95)) if len(derivative) else None
            ),
            "windows": len(errors),
            "endpoint_rmse_rad_s": (
                float(np.sqrt(np.mean(errors**2))) if len(errors) else None
            ),
            "endpoint_p95_abs_rad_s": (
                float(np.percentile(np.abs(errors), 95)) if len(errors) else None
            ),
        }
    return aggregate, per_run


def main():
    options = arguments()
    train = [load(path) for path in options.train]
    validation = [load(path) for path in options.validate]
    coefficients, optimizer = fit(train, options.minimum_damping)
    horizon = max(1, int(round(options.horizon / 0.02)))
    train_metrics, train_runs = metrics(
        train, coefficients, horizon, options.stride
    )
    validation_metrics, validation_runs = metrics(
        validation, coefficients, horizon, options.stride
    )
    strict_pass = all(
        validation_metrics[zone]["endpoint_rmse_rad_s"] is not None
        and validation_metrics[zone]["endpoint_rmse_rad_s"] <= 0.05
        for zone in options.required_zones
    )
    bounded_candidate = (
        not strict_pass
        and all(coefficients[:4] >= options.minimum_damping - 1.0e-9)
        and all(
            validation_metrics[zone]["endpoint_p95_abs_rad_s"] is not None
            and validation_metrics[zone]["endpoint_p95_abs_rad_s"] <= 0.25
            for zone in options.required_zones
        )
    )
    status = "PASS" if strict_pass else "BOUNDED_CANDIDATE" if bounded_candidate else "FAIL"
    result = {
        "model": "standard_vtol_stable_pitch_rate_lpv_v1",
        "equation": "q_dot=-a(V,lambda)q+b(V,lambda)q_sp+h(lambda)(d0+d_alpha*alpha+d_pitch*pitch)",
        "feature_names": FEATURE_NAMES,
        "coefficients": coefficients.tolist(),
        "speed_nodes_mps": [0.0, 20.0],
        "allocation_nodes_mu": [0.0, 1.0],
        "minimum_damping_per_s": options.minimum_damping,
        "horizon_s": options.horizon,
        "required_zones": options.required_zones,
        "training_csv": [str(path) for path in options.train],
        "validation_csv": [str(path) for path in options.validate],
        "training": train_metrics,
        "validation": validation_metrics,
        "training_runs": train_runs,
        "validation_runs": validation_runs,
        "optimizer_success": bool(optimizer.success),
        "optimizer_message": optimizer.message,
        "status": status,
    }
    options.output.mkdir(parents=True, exist_ok=True)
    (options.output / "pitch_rate_lpv.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8"
    )
    lines = [
        "# Stable Standard VTOL pitch-rate LPV identification",
        "",
        "The model is Markov, all scheduled damping nodes are constrained positive, and future actuator outputs are never inputs.",
        "",
        f"- coefficients: `{coefficients.tolist()}`",
        f"- optimizer: `{optimizer.message}`",
        "",
        "| split | zone | qdot RMSE / p95 [rad/s²] | 0.5 s endpoint RMSE / p95 [rad/s] |",
        "|---|---|---:|---:|",
    ]
    def formatted(value):
        return "n/a" if value is None else f"{value:.4f}"

    for split, values in (("train", train_metrics), ("validation", validation_metrics)):
        for zone in ZONES:
            value = values[zone]
            lines.append(
                f"| {split} | {zone} | {formatted(value['q_dot_rmse_rad_s2'])} / "
                f"{formatted(value['q_dot_p95_abs_rad_s2'])} | "
                f"{formatted(value['endpoint_rmse_rad_s'])} / "
                f"{formatted(value['endpoint_p95_abs_rad_s'])} |"
            )
    lines.extend([
        "",
        "`BOUNDED_CANDIDATE` is not a strict identification PASS. It may be used only with the reported uncertainty envelope in robust offline simulations.",
        "",
        f"PITCH_RATE_LPV={status}",
    ])
    (options.output / "pitch_rate_lpv_report.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )
    print((options.output / "pitch_rate_lpv_report.md").read_text(encoding="utf-8"))
    return 0 if status != "FAIL" else 2


if __name__ == "__main__":
    raise SystemExit(main())
