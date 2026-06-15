"""Run closed-loop offline NMPC transition simulation for a 6DOF VTOL."""

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


def make_initial_state(model: StandardVtolModel) -> np.ndarray:
    x = np.zeros(model.nx, dtype=float)
    x[3] = 0.05
    x[6] = 1.0
    return x


def make_forward_flight_reference(model: StandardVtolModel) -> np.ndarray:
    x_ref = np.zeros(model.nx, dtype=float)
    x_ref[0] = 120.0
    x_ref[3] = 16.0
    x_ref[6] = 1.0
    return x_ref


def run_simulation(
    total_time: float = 6.0,
    dt: float = 0.15,
    horizon_steps: int = 18,
) -> dict[str, np.ndarray]:
    model = StandardVtolModel()
    mpc = StandardVtolNMPC(model, MpcConfig(dt=dt, horizon_steps=horizon_steps))

    steps = int(round(total_time / dt))
    x = make_initial_state(model)
    x_ref = make_forward_flight_reference(model)

    state_history = np.zeros((steps + 1, model.nx))
    control_history = np.zeros((steps, model.nu))
    objective_history = np.zeros(steps)
    status_history: list[str] = []
    time = np.arange(steps + 1, dtype=float) * dt

    state_history[0] = x
    previous_solution = None
    horizon_duration = dt * horizon_steps

    for k in range(steps):
        local_ref = x_ref.copy()
        # Constant-acceleration reference: distance = average speed * horizon time.
        local_ref[0] = x[0] + 0.5 * (x[3] + x_ref[3]) * horizon_duration
        local_ref[1] = 0.0
        local_ref[2] = 0.0

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

        euler_deg = np.rad2deg(model.quaternion_to_euler(x[6:10]))
        print(
            f"{k + 1:03d}/{steps} "
            f"p=[{x[0]:6.1f}, {x[1]:5.1f}, {x[2]:5.1f}] m "
            f"v=[{x[3]:5.1f}, {x[4]:5.1f}, {x[5]:5.1f}] m/s "
            f"rpy=[{euler_deg[0]:5.1f}, {euler_deg[1]:5.1f}, {euler_deg[2]:5.1f}] deg "
            f"u=[{u[0]:5.1f}, {u[1]:5.1f}, {u[2]:5.2f}, {u[3]:5.2f}, {u[4]:5.2f}] "
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
    np.savez(output_dir / "transition_6dof_results.npz", **results)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--total-time", type=float, default=6.0)
    parser.add_argument("--dt", type=float, default=0.15)
    parser.add_argument("--horizon-steps", type=int, default=18)
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
    print(f"Saved results to {args.output_dir / 'transition_6dof_results.npz'}")


if __name__ == "__main__":
    main()
