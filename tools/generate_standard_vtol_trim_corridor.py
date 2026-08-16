#!/usr/bin/env python3
"""Generate the validated Standard VTOL transition trim corridor."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

import matplotlib
import numpy as np
import yaml


matplotlib.use("Agg")
import matplotlib.pyplot as plt


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "px4_mpc"))

from px4_mpc.models.standard_vtol_trim import StandardVtolTrimSolver  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--maximum-airspeed", type=float, default=22.0)
    parser.add_argument("--airspeed-step", type=float, default=1.0)
    parser.add_argument("--pitch-grid-step", type=float, default=0.25)
    parser.add_argument(
        "--output", type=Path, default=Path("results/standard_vtol_trim_corridor")
    )
    options = parser.parse_args()
    if options.maximum_airspeed <= 0.0 or options.airspeed_step <= 0.0:
        raise SystemExit("airspeed bounds and step must be positive")

    airspeeds = np.arange(
        0.0, options.maximum_airspeed + 0.5 * options.airspeed_step,
        options.airspeed_step,
    )
    solver = StandardVtolTrimSolver()
    corridor = solver.corridor(airspeeds, options.pitch_grid_step)
    options.output.mkdir(parents=True, exist_ok=True)

    fields = (
        "airspeed_m_s", "pitch_deg", "collective_lift", "pusher", "elevator_deg",
        "acceleration_x_m_s2", "acceleration_z_m_s2",
        "required_pitch_acceleration_rad_s2",
    )
    with (options.output / "trim_corridor.csv").open(
        "w", newline="", encoding="utf-8"
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for point in corridor:
            writer.writerow(
                {
                    "airspeed_m_s": point.airspeed,
                    "pitch_deg": np.rad2deg(point.pitch),
                    "collective_lift": point.collective_lift,
                    "pusher": point.pusher,
                    "elevator_deg": np.rad2deg(point.elevator),
                    "acceleration_x_m_s2": point.acceleration_world[0],
                    "acceleration_z_m_s2": point.acceleration_world[2],
                    "required_pitch_acceleration_rad_s2": point.required_pitch_acceleration,
                }
            )

    data = {
        "model": "StandardVtolGazeboModel",
        "controller_architecture": "NMPC commands collective/pusher/body-rates; PX4 closes rate loop",
        "pitch_convention": "Gazebo FLU rotation about +Y; negative is nose-up",
        "surface_trim": "diagnostic equilibrium value; PX4 rate loop, not NMPC, commands it",
        "points": [
            {
                "airspeed_m_s": point.airspeed,
                "pitch_rad": point.pitch,
                "collective_lift": point.collective_lift,
                "pusher": point.pusher,
                "elevator_rad": point.elevator,
            }
            for point in corridor
        ],
    }
    (options.output / "trim_corridor.yaml").write_text(
        yaml.safe_dump(data, sort_keys=False), encoding="utf-8"
    )

    speed = np.array([point.airspeed for point in corridor])
    figure, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    axes[0].plot(speed, [np.rad2deg(point.pitch) for point in corridor], marker="o")
    axes[0].set_ylabel("Pitch [deg]")
    axes[1].plot(speed, [point.collective_lift for point in corridor], marker="o")
    axes[1].set_ylabel("Collective lift")
    axes[2].plot(speed, [point.pusher for point in corridor], marker="o")
    axes[2].set_ylabel("Pusher")
    axes[3].plot(speed, [np.rad2deg(point.elevator) for point in corridor], marker="o")
    axes[3].set_ylabel("Elevator [deg]")
    axes[3].set_xlabel("Airspeed [m/s]")
    for axis in axes:
        axis.grid(True)
    figure.tight_layout()
    figure.savefig(options.output / "trim_corridor.png", dpi=150)
    plt.close(figure)

    maximum_residual = max(
        max(abs(point.acceleration_world[0]), abs(point.acceleration_world[2]))
        for point in corridor
    )
    print(f"Generated {len(corridor)} trim points")
    print(f"Maximum translational residual: {maximum_residual:.3e} m/s^2")
    print(f"Output: {options.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
