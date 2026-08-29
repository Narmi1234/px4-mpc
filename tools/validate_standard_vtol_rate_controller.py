#!/usr/bin/env python3
"""Validate the reduced PX4 MC/FW rate-controller equations on extracted CSVs."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "px4_mpc"))

from px4_mpc.models.standard_vtol_rate_control import StandardVtolRateControlModel  # noqa: E402


AXES = ("p", "q", "r")


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, nargs="+")
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def vector(row, prefix):
    return np.asarray([float(row[f"{prefix}_{axis}"]) for axis in AXES])


def xyz_vector(row, prefix):
    return np.asarray([float(row[f"{prefix}_{axis}"]) for axis in ("x", "y", "z")])


def load(paths):
    samples = []
    for path in paths:
        with path.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
        required = {
            "valid", "zone", "airspeed_mps", "fw_filtered_airspeed_mps",
            *(f"rate_sp_{axis}" for axis in AXES),
            *(f"omega_{axis}" for axis in AXES),
            *(f"omega_dot_{axis}" for axis in AXES),
            *(f"mc_torque_{axis}" for axis in ("x", "y", "z")),
            *(f"fw_torque_{axis}" for axis in ("x", "y", "z")),
            *(f"mc_integrator_{axis}" for axis in AXES),
            *(f"fw_integrator_{axis}" for axis in AXES),
            *(f"fw_compression_{axis}" for axis in AXES),
        }
        missing = required - (set(rows[0]) if rows else set())
        if missing:
            raise SystemExit(f"{path} missing columns: {', '.join(sorted(missing))}")
        for row in rows:
            if int(row["valid"]) == 1:
                row["source"] = str(path)
                samples.append(row)
    return samples


def metrics(measured, predicted):
    measured = np.asarray(measured)
    predicted = np.asarray(predicted)
    residual = predicted - measured
    centered_measured = measured - np.mean(measured, axis=0)
    centered_predicted = predicted - np.mean(predicted, axis=0)
    denominator = np.sqrt(
        np.sum(centered_measured**2, axis=0) * np.sum(centered_predicted**2, axis=0)
    )
    correlation = np.divide(
        np.sum(centered_measured * centered_predicted, axis=0),
        denominator,
        out=np.zeros(3),
        where=denominator > 1.0e-12,
    )
    return {
        "samples": len(measured),
        "rmse": np.sqrt(np.mean(residual**2, axis=0)),
        "p95_abs": np.percentile(np.abs(residual), 95, axis=0),
        "correlation": correlation,
    }


def main():
    options = arguments()
    rows = load(options.csv)
    model = StandardVtolRateControlModel()
    grouped = {
        (controller, zone): {"measured": [], "predicted": []}
        for controller in ("mc", "fw") for zone in ("mc", "blend", "fw")
    }
    for row in rows:
        zone = row["zone"]
        if zone not in ("mc", "blend", "fw"):
            continue
        rate = vector(row, "omega")
        rate_sp = vector(row, "rate_sp")
        acceleration = vector(row, "omega_dot")
        mc = model.mc_torque(
            rate, rate_sp, acceleration, vector(row, "mc_integrator")
        )
        fw = model.fw_torque(
            rate, rate_sp, acceleration, vector(row, "fw_integrator"),
            float(row["fw_filtered_airspeed_mps"]), vector(row, "fw_compression"),
        )
        grouped[("mc", zone)]["measured"].append(xyz_vector(row, "mc_torque"))
        grouped[("mc", zone)]["predicted"].append(mc)
        grouped[("fw", zone)]["measured"].append(xyz_vector(row, "fw_torque"))
        grouped[("fw", zone)]["predicted"].append(fw)

    values = {key: metrics(**data) for key, data in grouped.items() if data["measured"]}
    options.output.mkdir(parents=True, exist_ok=True)
    lines = [
        "# PX4 rate-controller equation validation",
        "",
        "Prediction uses logged rate, rate setpoint, angular acceleration, integrator, airspeed and FW gain-compression. It does not use logged torque as an input.",
        "",
        "| controller | zone | samples | torque RMSE x/y/z | correlation x/y/z |",
        "|---|---|---:|---:|---:|",
    ]
    for controller in ("mc", "fw"):
        for zone in ("mc", "blend", "fw"):
            value = values[(controller, zone)]
            lines.append(
                f"| {controller} | {zone} | {value['samples']}"
                f" | {' / '.join(f'{v:.5f}' for v in value['rmse'])}"
                f" | {' / '.join(f'{v:.3f}' for v in value['correlation'])} |"
            )
    report = options.output / "rate_controller_report.md"
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"report={report.resolve()}")
    for key, value in values.items():
        print(key, "rmse", value["rmse"], "correlation", value["correlation"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
