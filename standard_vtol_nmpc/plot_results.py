"""Plot saved standard VTOL transition simulation results."""

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

    fig, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=False)

    axes[0].plot(states[:, 0], states[:, 1], linewidth=2)
    axes[0].axhline(0.0, color="0.65", linestyle="--", linewidth=1)
    axes[0].set_ylabel("altitude z [m]")
    axes[0].set_xlabel("downrange x [m]")
    axes[0].grid(True)

    axes[1].plot(time, states[:, 2], label="vx")
    axes[1].plot(time, states[:, 3], label="vz")
    axes[1].axhline(16.0, color="0.35", linestyle="--", linewidth=1)
    axes[1].set_ylabel("velocity [m/s]")
    axes[1].legend(loc="best")
    axes[1].grid(True)

    axes[2].plot(time, np.rad2deg(states[:, 4]), label="theta")
    axes[2].plot(time, np.rad2deg(states[:, 5]), label="q")
    axes[2].set_ylabel("attitude [deg, deg/s]")
    axes[2].legend(loc="best")
    axes[2].grid(True)

    for idx, name in enumerate(model.control_names):
        axes[3].step(control_time, controls[:, idx], where="post", label=name)
    axes[3].set_ylabel("control")
    axes[3].set_xlabel("time [s]")
    axes[3].legend(loc="best")
    axes[3].grid(True)

    fig.suptitle(f"Standard VTOL NMPC transition, final objective {objective[-1]:.2f}")
    fig.tight_layout()

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
        default=default_dir / "transition_results.npz",
    )
    parser.add_argument("--output-file", type=Path, default=default_dir / "transition.png")
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    try:
        plot_results(args.results_file, None if args.show else args.output_file)
    except FileNotFoundError as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(2) from exc


if __name__ == "__main__":
    main()
