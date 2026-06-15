"""Plot saved 6DOF standard VTOL transition simulation results."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np

try:
    from .model import StandardVtolModel
except ImportError:  # Allows: python standard_vtol_nmpc/plot_results.py
    from model import StandardVtolModel


def plot_results(results_file: Path, output_file: Path | None = None) -> None:
    if not results_file.exists():
        raise FileNotFoundError(
            f"Results file does not exist: {results_file}\n"
            "Run the simulation first with: python -m standard_vtol_nmpc.simulate"
        )

    data = np.load(results_file, allow_pickle=True)
    import matplotlib.pyplot as plt

    time = data["time"]
    states = data["states"]
    controls = data["controls"]
    objective = data["objective"]

    model = StandardVtolModel()
    control_time = time[:-1]
    euler = np.array([model.quaternion_to_euler(q) for q in states[:, 6:10]])

    fig, axes = plt.subplots(8, 1, figsize=(12, 21), sharex=False)

    axes[0].plot(states[:, 0], states[:, 2], linewidth=2, label="x-z")
    axes[0].plot(states[:, 0], states[:, 1], linewidth=1.8, label="x-y")
    axes[0].axhline(0.0, color="0.65", linestyle="--", linewidth=1)
    axes[0].set_ylabel("cross/altitude [m]")
    axes[0].set_xlabel("downrange x [m]")
    axes[0].legend(loc="best")
    axes[0].grid(True)

    position_names = ("px", "py", "pz")
    position_colors = ("tab:blue", "tab:orange", "tab:green")
    for idx, (name, color) in enumerate(zip(position_names, position_colors), start=1):
        state_idx = idx - 1
        axes[idx].plot(time, states[:, state_idx], color=color, label=name)
        if name in ("py", "pz"):
            axes[idx].axhline(0.0, color="0.45", linestyle="--", linewidth=1)
        axes[idx].set_ylabel(f"{name} [m]")
        axes[idx].set_xlabel("time [s]")
        axes[idx].legend(loc="best")
        axes[idx].grid(True)

    axes[4].plot(time, states[:, 3], label="vx")
    axes[4].plot(time, states[:, 4], label="vy")
    axes[4].plot(time, states[:, 5], label="vz")
    axes[4].axhline(16.0, color="0.35", linestyle="--", linewidth=1)
    axes[4].set_ylabel("velocity [m/s]")
    axes[4].set_xlabel("time [s]")
    axes[4].legend(loc="best")
    axes[4].grid(True)

    axes[5].plot(time, np.rad2deg(euler[:, 0]), label="roll")
    axes[5].plot(time, np.rad2deg(euler[:, 1]), label="pitch")
    axes[5].plot(time, np.rad2deg(euler[:, 2]), label="yaw")
    axes[5].set_ylabel("attitude [deg]")
    axes[5].set_xlabel("time [s]")
    axes[5].legend(loc="best")
    axes[5].grid(True)

    axes[6].plot(time, np.rad2deg(states[:, 10]), label="wx")
    axes[6].plot(time, np.rad2deg(states[:, 11]), label="wy")
    axes[6].plot(time, np.rad2deg(states[:, 12]), label="wz")
    axes[6].set_ylabel("body rate [deg/s]")
    axes[6].set_xlabel("time [s]")
    axes[6].legend(loc="best")
    axes[6].grid(True)

    for idx, name in enumerate(model.control_names):
        axes[7].step(control_time, controls[:, idx], where="post", label=name)
    axes[7].set_ylabel("control")
    axes[7].set_xlabel("time [s]")
    axes[7].legend(loc="best")
    axes[7].grid(True)

    final_objective = objective[-1] if len(objective) else 0.0
    fig.suptitle(
        f"6DOF standard VTOL NMPC transition, final objective {final_objective:.2f}",
        y=0.995,
    )
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.985))

    if output_file is None:
        plt.show()
    else:
        output_file.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_file, dpi=160)
        print(f"Saved plot to {output_file}")


def main() -> None:
    parser = argparse.ArgumentParser()
    default_dir = Path(__file__).resolve().parent / "results"
    parser.add_argument(
        "--results-file",
        type=Path,
        default=default_dir / "transition_6dof_results.npz",
    )
    parser.add_argument("--output-file", type=Path, default=default_dir / "transition_6dof.png")
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    try:
        plot_results(args.results_file, None if args.show else args.output_file)
    except FileNotFoundError as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(2) from exc


if __name__ == "__main__":
    main()
