#!/usr/bin/env python3
"""Fit an airspeed/allocation-scheduled PX4 closed-loop body-rate model."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np
import yaml


AXES = ("p", "q", "r")
ZONES = ("mc", "blend", "fw")


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train", type=Path, nargs="+", required=True)
    parser.add_argument("--validate", type=Path, nargs="+", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--maximum-delay-ms", type=float, default=100.0)
    parser.add_argument("--ridge", type=float, default=1.0e-3)
    parser.add_argument("--rollout-seconds", type=float, default=0.5)
    return parser.parse_args()


def load_runs(paths, rate):
    required = {
        "valid", "time_s", "zone", "vtol_state", "airspeed_mps",
        "lift_fraction_proxy", "pitch_rad", "alpha_proxy_rad", "beta_proxy_rad",
        *(f"rate_sp_{axis}" for axis in AXES),
        *(f"omega_{axis}" for axis in AXES),
    }
    runs = []
    for path in paths:
        with path.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
        fields = set(rows[0]) if rows else set()
        missing = required - fields
        if missing:
            raise SystemExit(f"{path} missing columns: {', '.join(sorted(missing))}")
        rows = [row for row in rows if int(row["valid"]) == 1]
        time = np.asarray([float(row["time_s"]) for row in rows])
        if len(time) < 3 or not np.isclose(np.median(np.diff(time)), 1 / rate, rtol=0.02):
            raise SystemExit(f"{path} is empty or does not match --rate")
        runs.append({
            "path": str(path),
            "time": time,
            "zone": np.asarray([row["zone"] for row in rows]),
            "state": np.asarray([int(row["vtol_state"]) for row in rows]),
            "airspeed": np.asarray([float(row["airspeed_mps"]) for row in rows]),
            "lambda_proxy": np.asarray([float(row["lift_fraction_proxy"]) for row in rows]),
            "pitch": np.asarray([float(row["pitch_rad"]) for row in rows]),
            "alpha": np.asarray([float(row["alpha_proxy_rad"]) for row in rows]),
            "beta": np.asarray([float(row["beta_proxy_rad"]) for row in rows]),
            "sp": np.column_stack([
                np.asarray([float(row[f"rate_sp_{axis}"]) for row in rows]) for axis in AXES
            ]),
            "omega": np.column_stack([
                np.asarray([float(row[f"omega_{axis}"]) for row in rows]) for axis in AXES
            ]),
        })
    return runs


def scheduling(run, index):
    speed = np.clip(run["airspeed"][index], 0.0, 25.0) / 15.0
    lam = np.clip(run["lambda_proxy"][index], 0.0, 1.0)
    if not np.isfinite(lam):
        lam = 1.0 if run["state"][index] == 3 else 0.0 if run["state"][index] == 4 else 0.5
    mu = 1.0 - lam
    alpha = np.clip(run["alpha"][index], -0.7, 0.7)
    beta = np.clip(run["beta"][index], -0.7, 0.7)
    pitch = np.clip(run["pitch"][index], -0.7, 0.7)
    front = float(run["state"][index] == 1)
    back = float(run["state"][index] == 2)
    return speed, mu, alpha, beta, pitch, front, back


def feature(run, index, delay, omega=None):
    current = run["omega"][index] if omega is None else omega
    error = run["sp"][max(0, index - delay)] - current
    speed, mu, alpha, beta, pitch, front, back = scheduling(run, index)
    return np.r_[
        error,
        error * speed,
        error * mu,
        error * speed * mu,
        1.0,
        speed,
        speed * speed,
        mu,
        speed * mu,
        alpha,
        beta,
        pitch,
        speed * alpha,
        mu * alpha,
        front,
        back,
    ]


FEATURE_NAMES = [
    *(f"rate_error_{axis}" for axis in AXES),
    *(f"rate_error_{axis}_times_speed" for axis in AXES),
    *(f"rate_error_{axis}_times_mu" for axis in AXES),
    *(f"rate_error_{axis}_times_speed_mu" for axis in AXES),
    "bias", "speed", "speed_squared", "mu", "speed_mu", "alpha", "beta",
    "pitch", "speed_alpha", "mu_alpha", "front_transition", "back_transition",
]


def samples(runs, delay, rate):
    features, target, zones = [], [], []
    for run in runs:
        for k in range(delay, len(run["time"]) - 1):
            if run["time"][k + 1] - run["time"][k] > 0.03:
                continue
            values = feature(run, k, delay)
            derivative = rate * (run["omega"][k + 1] - run["omega"][k])
            if np.all(np.isfinite(values)) and np.all(np.isfinite(derivative)):
                features.append(values)
                target.append(derivative)
                zones.append(run["zone"][k])
    return np.asarray(features), np.asarray(target), np.asarray(zones)


def fit(features, target, zones, ridge):
    counts = {zone: max(1, np.count_nonzero(zones == zone)) for zone in ZONES}
    weights = np.asarray([1.0 / counts.get(zone, 1) for zone in zones])
    weights *= len(weights) / np.sum(weights)
    scale = np.maximum(np.std(features, axis=0), 1.0e-6)
    scale[12] = 1.0  # bias
    normalized = features / scale
    weighted = normalized * np.sqrt(weights[:, None])
    weighted_target = target * np.sqrt(weights[:, None])
    regularizer = ridge * np.eye(normalized.shape[1])
    regularizer[12, 12] = 0.0
    coefficients = np.linalg.solve(
        weighted.T @ weighted + regularizer, weighted.T @ weighted_target
    )
    return coefficients / scale[:, None]


def metrics(features, target, zones, coefficients):
    prediction = features @ coefficients
    residual = prediction - target
    result = {}
    for zone in ZONES:
        mask = zones == zone
        result[zone] = {
            "samples": int(np.count_nonzero(mask)),
            "derivative_rmse": np.sqrt(np.mean(residual[mask] ** 2, axis=0)),
            "derivative_p95_abs": np.percentile(np.abs(residual[mask]), 95, axis=0),
        }
    return result


def one_step_rmse(metrics_value, rate):
    return np.asarray(metrics_value["derivative_rmse"]) / rate


def rollout(runs, delay, rate, coefficients, horizon):
    errors = {zone: [] for zone in ZONES}
    for run in runs:
        for start in range(delay, len(run["time"]) - horizon, horizon):
            zone = run["zone"][start]
            if zone not in errors or not np.all(run["zone"][start:start + horizon + 1] == zone):
                continue
            predicted = run["omega"][start].copy()
            for offset in range(horizon):
                index = start + offset
                predicted += (feature(run, index, delay, predicted) @ coefficients) / rate
                errors[zone].append(predicted - run["omega"][index + 1])
    return {
        zone: {
            "samples": len(values),
            "rmse": np.sqrt(np.mean(np.asarray(values) ** 2, axis=0)) if values else np.full(3, np.nan),
        }
        for zone, values in errors.items()
    }


def local_stability(runs, coefficients, rate):
    # First 12 coefficient rows are four 3-vector rate-error blocks.
    block = [coefficients[3 * i:3 * (i + 1), :].T for i in range(4)]
    maxima = {zone: 0.0 for zone in ZONES}
    for run in runs:
        for k in range(0, len(run["time"]), 10):
            speed, mu, *_ = scheduling(run, k)
            gain = block[0] + speed * block[1] + mu * block[2] + speed * mu * block[3]
            radius = float(np.max(np.abs(np.linalg.eigvals(np.eye(3) - gain / rate))))
            zone = run["zone"][k]
            if zone in maxima:
                maxima[zone] = max(maxima[zone], radius)
    return maxima


def serializable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: serializable(item) for key, item in value.items()}
    return value


def main():
    options = arguments()
    train = load_runs(options.train, options.rate)
    validation = load_runs(options.validate, options.rate)
    maximum_delay = int(round(options.maximum_delay_ms * 1e-3 * options.rate))
    best = None
    for delay in range(maximum_delay + 1):
        x, y, zones = samples(train, delay, options.rate)
        coefficients = fit(x, y, zones, options.ridge)
        value = metrics(x, y, zones, coefficients)
        # Equal zone weighting prevents long hover segments hiding transition error.
        score = float(np.mean([np.mean(value[zone]["derivative_rmse"]) for zone in ZONES]))
        if best is None or score < best[0]:
            best = score, delay, coefficients
    _, delay, coefficients = best
    train_x, train_y, train_zones = samples(train, delay, options.rate)
    val_x, val_y, val_zones = samples(validation, delay, options.rate)
    train_metrics = metrics(train_x, train_y, train_zones, coefficients)
    validation_metrics = metrics(val_x, val_y, val_zones, coefficients)
    horizon = max(1, int(round(options.rollout_seconds * options.rate)))
    rollouts = rollout(validation, delay, options.rate, coefficients, horizon)
    spectral_radius = local_stability(train + validation, coefficients, options.rate)
    checks = {
        zone: {
            "local_discrete_spectral_radius_below_one": spectral_radius[zone] < 1.0,
            "validation_samples": validation_metrics[zone]["samples"] >= 100,
            "validation_rollout": rollouts[zone]["samples"] >= horizon,
        }
        for zone in ZONES
    }
    overall_pass = all(all(value.values()) for value in checks.values())
    result = serializable({
        "model": "standard_vtol_closed_loop_rate_lpv_v1",
        "equation": "omega_dot = K(V,mu)*(omega_sp-omega)+b(V,mu,alpha,beta,pitch,phase)",
        "sample_rate_hz": options.rate,
        "delay_samples": delay,
        "delay_ms": 1000 * delay / options.rate,
        "feature_names": FEATURE_NAMES,
        "coefficients_feature_by_output": coefficients,
        "training_csv": [str(path) for path in options.train],
        "validation_csv": [str(path) for path in options.validate],
        "train": train_metrics,
        "validation": validation_metrics,
        "validation_rollout": rollouts,
        "maximum_local_spectral_radius": spectral_radius,
        "checks": checks,
        "identification_gate": "PASS" if overall_pass else "FAIL",
    })
    options.output.mkdir(parents=True, exist_ok=True)
    yaml_path = options.output / "rate_lpv.yaml"
    yaml_path.write_text(yaml.safe_dump(result, sort_keys=False), encoding="utf-8")
    lines = [
        "# Standard VTOL LPV closed-loop rate identification",
        "",
        "`omega_dot = K(V,mu)(omega_sp-omega) + b(V,mu,alpha,beta,pitch,phase)`, where `mu=1-lambda_proxy`.",
        f"Selected command delay: `{1000 * delay / options.rate:.0f} ms` using training data only.",
        "",
        "| zone | validation one-step RMSE p/q/r | 0.5 s rollout RMSE p/q/r | max local spectral radius |",
        "|---|---:|---:|---:|",
    ]
    for zone in ZONES:
        one_step = one_step_rmse(validation_metrics[zone], options.rate)
        lines.append(
            f"| {zone} | {' / '.join(f'{v:.4f}' for v in one_step)}"
            f" | {' / '.join(f'{v:.4f}' for v in rollouts[zone]['rmse'])}"
            f" | {spectral_radius[zone]:.5f} |"
        )
    lines.extend([
        "",
        "This is still a grey-box candidate. Promotion additionally requires transient plots and physically sensible gain scheduling; a numerical PASS alone is insufficient.",
        "",
        f"**IDENTIFICATION_GATE={'PASS' if overall_pass else 'FAIL'}**",
    ])
    report_path = options.output / "fit_report.md"
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"report={report_path.resolve()}")
    print(f"delay={1000 * delay / options.rate:.0f}ms")
    for zone in ZONES:
        print(zone, "one_step_rmse", one_step_rmse(validation_metrics[zone], options.rate), "rollout", rollouts[zone]["rmse"], "rho", spectral_radius[zone])
    print(f"IDENTIFICATION_GATE={'PASS' if overall_pass else 'FAIL'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
