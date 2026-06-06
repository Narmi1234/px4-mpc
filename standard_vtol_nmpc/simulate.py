"""Run closed-loop offline NMPC transition simulation."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np

try:
    from .model import StandardVtolModel
    from .mpc_casadi import MpcConfig, StandardVtolNMPC
except ImportError:  # Allows: python standard_vtol_nmpc/simulate.py
    from model import StandardVtolModel
    from mpc_casadi import MpcConfig, StandardVtolNMPC


def run_simulation(
    total_time: float = 8.0,
    dt: float = 0.12,
    horizon_steps: int = 25,
) -> dict[str, np.ndarray]:
    model = StandardVtolModel()
    mpc = StandardVtolNMPC(model, MpcConfig(dt=dt, horizon_steps=horizon_steps))

    steps = int(round(total_time / dt))
    x = np.array([0.0, 0.0, 0.05, 0.0, 0.0, 0.0], dtype=float)
    x_ref = np.array([120.0, 0.0, 16.0, 0.0, 0.04, 0.0], dtype=float)

    state_history = np.zeros((steps + 1, model.nx))
    control_history = np.zeros((steps, model.nu))
    objective_history = np.zeros(steps)
    status_history: list[str] = []
    time = np.arange(steps + 1, dtype=float) * dt

    state_history[0] = x
    previous_solution = None

    for k in range(steps):
        # Move the position target forward with the vehicle so velocity and
        # altitude dominate the transition behavior.
        local_ref = x_ref.copy()
        local_ref[0] = x[0] + 35.0

        solution = mpc.solve(x, local_ref, previous_solution)
        u = np.asarray(solution["u0"], dtype=float)

        x = model.rk4_step(x, u, dt)
        state_history[k + 1] = x
        control_history[k] = u
        objective_history[k] = float(solution["objective"])
        status_history.append(str(solution["status"]))
        previous_solution = {
            "x_pred": np.asarray(solution["x_pred"], dtype=float),
            "u_pred": np.asarray(solution["u_pred"], dtype=float),
        }

        print(
            f"{k + 1:03d}/{steps} "
            f"vx={x[2]:6.2f} m/s z={x[1]:6.2f} m "
            f"theta={np.rad2deg(x[4]):6.2f} deg "
            f"u=[{u[0]:5.1f}, {u[1]:5.1f}, {u[2]:5.2f}] "
            f"{solution['status']}"
        )

    return {
        "time": time,
        "states": state_history,
        "controls": control_history,
        "objective": objective_history,
        "status": np.array(status_history),
    }


def save_results(results: dict[str, np.ndarray], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    np.savez(output_dir / "transition_results.npz", **results)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--total-time", type=float, default=8.0)
    parser.add_argument("--dt", type=float, default=0.12)
    parser.add_argument("--horizon-steps", type=int, default=25)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "results",
    )
    args = parser.parse_args()

    try:
        results = run_simulation(
            total_time=args.total_time,
            dt=args.dt,
            horizon_steps=args.horizon_steps,
        )
    except ModuleNotFoundError as exc:
        if "CasADi" not in str(exc) and "casadi" not in str(exc):
            raise
        print(str(exc), file=sys.stderr)
        raise SystemExit(2) from exc

    save_results(results, args.output_dir)
    print(f"Saved results to {args.output_dir / 'transition_results.npz'}")


if __name__ == "__main__":
    main()
