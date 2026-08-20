#!/usr/bin/env python3
"""Offline closed-loop rehearsal of Gate A: 3 m/s MC pusher feedback."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from px4_mpc.controllers.standard_vtol_nmpc import StandardVtolNmpc
from px4_mpc.controllers.standard_vtol_output import (
    limit_pusher_forward_command,
    vertical_hover_lift,
)
from px4_mpc.models.mc_forward_profile import (
    McForwardProfile,
    pusher_forward_feedforward,
    pusher_forward_reference_state,
)
from px4_mpc.models.standard_vtol_gz_model import StandardVtolTransitionRateModel


def rk4_step(model, state, control, parameters, dt):
    """Integrate the reduced NumPy plant by one controller sample."""
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
    """Build the exact Gate A horizon for yaw zero."""
    direction = np.array([1.0, 0.0])
    samples = [
        profile.sample(profile_time + stage * controller.dt)
        for stage in range(controller.N + 1)
    ]
    x_ref = np.vstack(
        [
            pusher_forward_reference_state(hold_state, direction, sample)
            for sample in samples
        ]
    )
    u_ref = np.zeros((controller.N, 5))
    u_ref[:, 0] = controller.model.plant.hover_command
    u_ref[:, 1] = [
        pusher_forward_feedforward(
            controller.model.plant,
            sample.speed,
            sample.acceleration,
        )
        for sample in samples[:-1]
    ]
    parameters = np.zeros((controller.N + 1, 4))
    return x_ref, u_ref, parameters


def main() -> None:
    """Run the Gate A rehearsal and enforce its live acceptance envelope."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=20.5)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("results/standard_vtol_pusher_forward_offline"),
    )
    arguments = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    controller = StandardVtolNmpc(
        build_directory=root / "build/standard_vtol_nmpc"
    )
    plant = StandardVtolTransitionRateModel(controller.model.plant)
    profile = McForwardProfile(
        target_speed=3.0,
        acceleration=0.75,
        hold_seconds=2.0,
        start_delay_seconds=2.0,
    )
    state = plant.hover_state()
    state[2] = 20.0
    hold_state = state.copy()
    command = plant.hover_control()
    states = [state.copy()]
    controls = []
    raw_controls = []
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
            command = limit_pusher_forward_command(
                command, requested, controller.dt
            )
            raw_controls.append(solution.control.copy())
        else:
            solver_failures += 1
            consecutive_failures += 1
            raw_controls.append(np.full(5, np.nan))
        if elapsed < 0.5:
            command = plant.hover_control()
        if consecutive_failures >= 3:
            break
        state = rk4_step(plant, state, command, parameters[0], controller.dt)
        states.append(state.copy())
        controls.append(command.copy())
        state_references.append(x_ref[0].copy())

    states = np.asarray(states)
    controls = np.asarray(controls)
    raw_controls = np.asarray(raw_controls)
    state_references = np.asarray(state_references)
    solve_times = np.asarray(solve_times)
    position_error = states[1:, 0:2] - state_references[:, 0:2]
    qw, qx, qy, qz = states[:, 6], states[:, 7], states[:, 8], states[:, 9]
    roll = np.arctan2(
        2.0 * (qw * qx + qy * qz),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )
    pitch = np.arcsin(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0))
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
        "max_vertical_speed_m_s": float(np.max(np.abs(states[:, 5]))),
        "max_tilt_deg": float(
            np.degrees(np.max(np.maximum(np.abs(roll), np.abs(pitch))))
        ),
        "max_pusher": float(np.max(controls[:, 1])),
        "final_pusher": float(abs(controls[-1, 1])),
        "solve_time_p99_ms": float(1000.0 * np.percentile(solve_times, 99)),
    }
    passed = (
        len(controls) == round(arguments.duration / controller.dt)
        and metrics["solver_failures"] == 0
        and 2.5 <= metrics["max_speed_m_s"] <= 3.5
        and metrics["final_speed_m_s"] <= 0.35
        and metrics["final_position_error_m"] <= 1.0
        and metrics["max_tracking_error_m"] <= 2.0
        and metrics["max_altitude_error_m"] <= 0.30
        and metrics["max_vertical_speed_m_s"] <= 0.75
        and metrics["max_tilt_deg"] <= 10.0
        and 0.05 <= metrics["max_pusher"] <= 0.10 + 1.0e-12
        and metrics["final_pusher"] <= 0.005
        and metrics["solve_time_p99_ms"] <= 40.0
    )
    arguments.output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        arguments.output / "offline_pusher_forward_gate.npz",
        states=states,
        controls=controls,
        raw_controls=raw_controls,
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
