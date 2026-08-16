#!/usr/bin/env python3
"""Offline closed-loop rehearsal of the guarded 2 m/s MC-forward gate."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from px4_mpc.controllers.standard_vtol_nmpc import StandardVtolNmpc
from px4_mpc.controllers.standard_vtol_output import (
    limit_mc_command,
    vertical_hover_lift,
)
from px4_mpc.models.mc_forward_profile import (
    McForwardProfile,
    mc_forward_reference_state,
)
from px4_mpc.models.standard_vtol_gz_model import StandardVtolTransitionRateModel


def rk4_step(model, state, control, parameters, dt):
    """Integrate the NumPy rate-controlled plant by one controller sample."""
    def dynamics(value):
        return model.derivative(value, control, parameters[:3], parameters[3])

    k1 = dynamics(state)
    k2 = dynamics(state + 0.5 * dt * k1)
    k3 = dynamics(state + 0.5 * dt * k2)
    k4 = dynamics(state + dt * k3)
    result = state + dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
    result[6:10] /= np.linalg.norm(result[6:10])
    return result


def references(controller, profile, hold_state, profile_time):
    """Build the exact moving horizon used by the live node for yaw zero."""
    direction = np.array([1.0, 0.0])
    x_ref = np.vstack(
        [
            mc_forward_reference_state(
                hold_state,
                direction,
                profile.sample(profile_time + stage * controller.dt),
                controller.model.plant.gravity,
            )
            for stage in range(controller.N + 1)
        ]
    )
    u_ref = np.zeros((controller.N, 5))
    u_ref[:, 0] = controller.model.plant.hover_command
    parameters = np.zeros((controller.N + 1, 4))
    return x_ref, u_ref, parameters


def main() -> None:
    """Run the offline gate, save its arrays, and enforce live criteria."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("results/standard_vtol_mc_forward_offline"),
    )
    arguments = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    controller = StandardVtolNmpc(
        build_directory=root / "build/standard_vtol_nmpc"
    )
    plant = StandardVtolTransitionRateModel(controller.model.plant)
    profile = McForwardProfile()
    state = plant.hover_state()
    state[2] = 20.0
    hold_state = state.copy()
    command = plant.hover_control()
    states = [state.copy()]
    controls = []
    state_references = []
    solve_times = []
    solver_failures = 0
    consecutive_failures = 0

    for index in range(round(arguments.duration / controller.dt)):
        elapsed = index * controller.dt
        profile_time = max(0.0, elapsed - 0.5)
        x_ref, u_ref, parameters = references(
            controller, profile, hold_state, profile_time
        )
        solution = controller.solve(state, x_ref, u_ref, parameters)
        valid = solution.status == 0 and np.all(np.isfinite(solution.control))
        solve_times.append(solution.solve_time)
        if valid:
            consecutive_failures = 0
            requested = solution.control.copy()
            requested[0] = vertical_hover_lift(
                plant.plant,
                state[2] - hold_state[2],
                state[5],
            )
            command = limit_mc_command(command, requested, controller.dt)
        else:
            solver_failures += 1
            consecutive_failures += 1
        if elapsed < 0.5:
            command = plant.hover_control()
        if consecutive_failures >= 3:
            break
        state = rk4_step(
            plant, state, command, parameters[0], controller.dt
        )
        states.append(state.copy())
        controls.append(command.copy())
        state_references.append(x_ref[0].copy())

    states = np.asarray(states)
    controls = np.asarray(controls)
    state_references = np.asarray(state_references)
    solve_times = np.asarray(solve_times)
    position_error = states[1:, 0:2] - state_references[:, 0:2]
    pitch = 2.0 * np.arctan2(
        np.abs(states[:, 8]), np.maximum(1.0e-12, np.abs(states[:, 6]))
    )
    metrics = {
        "solver_failures": solver_failures,
        "max_speed_m_s": float(np.max(states[:, 3])),
        "final_speed_m_s": float(np.linalg.norm(states[-1, 3:5])),
        "final_position_error_m": float(
            np.linalg.norm(states[-1, 0:2] - state_references[-1, 0:2])
        ),
        "max_tracking_error_m": float(
            np.max(np.linalg.norm(position_error, axis=1))
        ),
        "max_altitude_error_m": float(
            np.max(np.abs(states[:, 2] - hold_state[2]))
        ),
        "max_tilt_deg": float(np.degrees(np.max(pitch))),
        "max_abs_pusher": float(np.max(np.abs(controls[:, 1]))),
        "solve_time_p99_ms": float(1000.0 * np.percentile(solve_times, 99)),
    }
    passed = (
        len(controls) == round(arguments.duration / controller.dt)
        and metrics["solver_failures"] == 0
        and 1.6 <= metrics["max_speed_m_s"] <= 2.7
        and metrics["final_speed_m_s"] <= 0.35
        and metrics["final_position_error_m"] <= 0.75
        and metrics["max_tracking_error_m"] <= 1.5
        and metrics["max_altitude_error_m"] <= 0.5
        and metrics["max_tilt_deg"] <= 25.0
        and metrics["max_abs_pusher"] == 0.0
    )
    arguments.output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        arguments.output / "offline_mc_forward_gate.npz",
        states=states,
        controls=controls,
        state_references=state_references,
        solve_times=solve_times,
    )
    for name, value in metrics.items():
        print(f"{name}={value:.6g}")
    print(f"offline_gate={'PASS' if passed else 'FAIL'}")
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
