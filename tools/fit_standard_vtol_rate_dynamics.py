#!/usr/bin/env python3
"""Fit and validate zone-dependent closed-loop PX4 body-rate dynamics."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np
import yaml


AXES = ("p", "q", "r")
ZONES = ("mc", "blend", "fw")


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train", type=Path, nargs="+", required=True)
    parser.add_argument("--validate", type=Path, nargs="+", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--maximum-delay-ms", type=float, default=100.0)
    parser.add_argument("--ridge", type=float, default=1.0e-4)
    parser.add_argument("--rollout-seconds", type=float, default=0.5)
    return parser.parse_args()


def load_runs(paths: list[Path], rate: float):
    runs = []
    required = {
        "valid", "time_s", "zone", "airspeed_mps", "lift_fraction_proxy",
        *(f"rate_sp_{axis}" for axis in AXES),
        *(f"omega_{axis}" for axis in AXES),
    }
    for path in paths:
        with path.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
            fields = set(rows[0]) if rows else set()
        missing = required - fields
        if missing:
            raise SystemExit(f"{path} missing columns: {', '.join(sorted(missing))}")
        selected = [row for row in rows if int(row["valid"]) == 1]
        if len(selected) < 3:
            raise SystemExit(f"{path} has too few valid samples")
        time = np.asarray([float(row["time_s"]) for row in selected])
        median_dt = float(np.median(np.diff(time)))
        if not np.isclose(median_dt, 1.0 / rate, rtol=0.02, atol=1.0e-5):
            raise SystemExit(
                f"{path} sample interval {median_dt:.6f}s does not match --rate {rate}"
            )
        runs.append({
            "path": str(path),
            "time": time,
            "zone": np.asarray([row["zone"] for row in selected]),
            "airspeed": np.asarray([float(row["airspeed_mps"]) for row in selected]),
            "lift_fraction": np.asarray([float(row["lift_fraction_proxy"]) for row in selected]),
            "sp": np.column_stack([
                np.asarray([float(row[f"rate_sp_{axis}"]) for row in selected])
                for axis in AXES
            ]),
            "omega": np.column_stack([
                np.asarray([float(row[f"omega_{axis}"]) for row in selected])
                for axis in AXES
            ]),
        })
    return runs


def pairs(runs, zone: str, delay: int):
    features, targets = [], []
    for run in runs:
        for k in range(delay, len(run["time"]) - 1):
            if run["zone"][k] != zone or run["zone"][k + 1] != zone:
                continue
            if run["time"][k + 1] - run["time"][k] > 0.03:
                continue
            features.append(np.r_[run["omega"][k], run["sp"][k - delay], 1.0])
            targets.append(run["omega"][k + 1])
    if not features:
        return np.empty((0, 7)), np.empty((0, 3))
    return np.asarray(features), np.asarray(targets)


def ridge_fit(features: np.ndarray, target: np.ndarray, ridge: float):
    scale = np.maximum(np.std(features, axis=0), 1.0e-6)
    scale[-1] = 1.0
    normalized = features / scale
    regularizer = ridge * np.eye(normalized.shape[1])
    regularizer[-1, -1] = 0.0
    coefficients = np.linalg.solve(
        normalized.T @ normalized + regularizer, normalized.T @ target
    )
    return coefficients / scale[:, None]


def fit_zone(runs, zone, maximum_delay, ridge):
    best = None
    for delay in range(maximum_delay + 1):
        features, target = pairs(runs, zone, delay)
        if len(features) < 100:
            continue
        coefficients = ridge_fit(features, target, ridge)
        prediction = features @ coefficients
        rmse = np.sqrt(np.mean((prediction - target) ** 2))
        candidate = (rmse, delay, coefficients, len(features))
        if best is None or candidate[0] < best[0]:
            best = candidate
    if best is None:
        raise SystemExit(f"Insufficient training excitation/samples for zone {zone}")
    _, delay, coefficients, count = best
    # features=[omega(3), setpoint(3), 1], target=omega_next(3)
    return {
        "delay": delay,
        "A": coefficients[0:3, :].T,
        "B": coefficients[3:6, :].T,
        "c": coefficients[6, :],
        "samples": count,
    }


def one_step_metrics(runs, zone, model):
    features, target = pairs(runs, zone, model["delay"])
    coefficients = np.vstack((model["A"].T, model["B"].T, model["c"]))
    prediction = features @ coefficients
    residual = prediction - target
    hold_residual = features[:, 0:3] - target
    return {
        "samples": len(target),
        "rmse": np.sqrt(np.mean(residual**2, axis=0)),
        "p95_abs": np.percentile(np.abs(residual), 95, axis=0),
        "hold_rmse": np.sqrt(np.mean(hold_residual**2, axis=0)),
    }


def rollout_metrics(runs, zone, model, horizon):
    errors = []
    delay = model["delay"]
    for run in runs:
        for start in range(delay, len(run["time"]) - horizon, horizon):
            if not np.all(run["zone"][start:start + horizon + 1] == zone):
                continue
            predicted = run["omega"][start].copy()
            for offset in range(horizon):
                sp_index = max(0, start + offset - delay)
                predicted = (
                    model["A"] @ predicted
                    + model["B"] @ run["sp"][sp_index]
                    + model["c"]
                )
                errors.append(predicted - run["omega"][start + offset + 1])
    if not errors:
        return {"samples": 0, "rmse": np.full(3, np.nan)}
    errors = np.asarray(errors)
    return {"samples": len(errors), "rmse": np.sqrt(np.mean(errors**2, axis=0))}


def serializable(values):
    if isinstance(values, np.ndarray):
        return values.tolist()
    if isinstance(values, np.generic):
        return values.item()
    if isinstance(values, dict):
        return {key: serializable(value) for key, value in values.items()}
    return values


def main() -> int:
    options = arguments()
    if options.rate <= 0 or options.rollout_seconds <= 0:
        raise SystemExit("--rate and --rollout-seconds must be positive")
    train = load_runs(options.train, options.rate)
    validation = load_runs(options.validate, options.rate)
    maximum_delay = int(round(options.maximum_delay_ms * 1.0e-3 * options.rate))
    horizon = max(1, int(round(options.rollout_seconds * options.rate)))

    models, train_metrics, validation_metrics, rollouts = {}, {}, {}, {}
    for zone in ZONES:
        model = fit_zone(train, zone, maximum_delay, options.ridge)
        models[zone] = model
        train_metrics[zone] = one_step_metrics(train, zone, model)
        validation_metrics[zone] = one_step_metrics(validation, zone, model)
        rollouts[zone] = rollout_metrics(validation, zone, model, horizon)

    result = {
        "model": "standard_vtol_closed_loop_rate_dynamics_v1",
        "equation": "omega[k+1] = A_zone*omega[k] + B_zone*omega_sp[k-delay] + c_zone",
        "sample_rate_hz": options.rate,
        "training_csv": [str(path) for path in options.train],
        "validation_csv": [str(path) for path in options.validate],
        "zones": {
            zone: {
                "delay_samples": models[zone]["delay"],
                "delay_ms": 1000.0 * models[zone]["delay"] / options.rate,
                "training_samples": models[zone]["samples"],
                "A": models[zone]["A"],
                "B": models[zone]["B"],
                "c": models[zone]["c"],
                "train_one_step": train_metrics[zone],
                "validation_one_step": validation_metrics[zone],
                "validation_rollout": rollouts[zone],
            }
            for zone in ZONES
        },
    }
    result = serializable(result)
    checks = {}
    for zone in ZONES:
        model = models[zone]
        validation_value = validation_metrics[zone]
        stable = bool(np.max(np.abs(np.linalg.eigvals(model["A"]))) < 1.0)
        positive_same_axis = bool(np.all(np.diag(model["B"]) > 0.0))
        improves_hold = bool(
            np.mean(validation_value["rmse"])
            < np.mean(validation_value["hold_rmse"])
        )
        has_rollout = bool(rollouts[zone]["samples"] >= horizon)
        checks[zone] = {
            "stable_discrete_dynamics": stable,
            "positive_same_axis_authority": positive_same_axis,
            "improves_validation_hold_baseline": improves_hold,
            "has_validation_rollout": has_rollout,
        }
    result["promotion_checks"] = checks
    overall_pass = all(all(zone_checks.values()) for zone_checks in checks.values())
    result["identification_gate"] = "PASS" if overall_pass else "FAIL"
    options.output.mkdir(parents=True, exist_ok=True)
    yaml_path = options.output / "rate_dynamics.yaml"
    yaml_path.write_text(yaml.safe_dump(result, sort_keys=False), encoding="utf-8")

    lines = [
        "# Standard VTOL closed-loop rate identification",
        "",
        "Model: `omega[k+1] = A omega[k] + B omega_sp[k-delay] + c`.",
        "The validation CSVs were not used to select delay or coefficients.",
        "",
        "| Zone | delay | train samples | validation samples | validation RMSE p/q/r | 0.5 s rollout RMSE p/q/r |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for zone in ZONES:
        validation_value = validation_metrics[zone]
        rollout = rollouts[zone]
        lines.append(
            f"| {zone} | {1000 * models[zone]['delay'] / options.rate:.0f} ms"
            f" | {models[zone]['samples']} | {validation_value['samples']}"
            f" | {' / '.join(f'{v:.4f}' for v in validation_value['rmse'])}"
            f" | {' / '.join(f'{v:.4f}' for v in rollout['rmse'])} |"
        )
    lines.extend([
        "",
        "## Promotion checks",
        "",
        "| Zone | stable A | positive diag(B) | beats hold baseline | validation rollout |",
        "|---|---:|---:|---:|---:|",
    ])
    for zone in ZONES:
        value = checks[zone]
        mark = lambda passed: "PASS" if passed else "FAIL"
        lines.append(
            f"| {zone} | {mark(value['stable_discrete_dynamics'])}"
            f" | {mark(value['positive_same_axis_authority'])}"
            f" | {mark(value['improves_validation_hold_baseline'])}"
            f" | {mark(value['has_validation_rollout'])} |"
        )
    lines.extend([
        "",
        "## Interpretation gate",
        "",
        "This candidate is an identification artifact, not automatically promoted to the OCP. Check positive same-axis command authority (`diag(B)`), stable discrete eigenvalues, improvement over zero-order hold, and pitch-rate transient plots before promotion.",
        "",
        f"**IDENTIFICATION_GATE={'PASS' if overall_pass else 'FAIL'}**",
    ])
    report_path = options.output / "fit_report.md"
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"parameters={yaml_path.resolve()}")
    print(f"report={report_path.resolve()}")
    for zone in ZONES:
        print(
            f"{zone}: delay={models[zone]['delay'] / options.rate:.3f}s "
            f"validation_rmse={validation_metrics[zone]['rmse']} "
            f"rollout_rmse={rollouts[zone]['rmse']}"
        )
    print(f"IDENTIFICATION_GATE={'PASS' if overall_pass else 'FAIL'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
