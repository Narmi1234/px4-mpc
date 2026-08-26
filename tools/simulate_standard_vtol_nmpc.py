#!/usr/bin/env python3
"""Offline closed-loop test of the Standard VTOL transition NMPC."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from px4_mpc.controllers.standard_vtol_nmpc import StandardVtolNmpc
from px4_mpc.models.standard_vtol_gz_model import StandardVtolTransitionRateModel


def load_corridor(path: Path) -> dict[str, np.ndarray]:
    """Load the generated trim corridor into interpolation arrays."""
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    return {
        "speed": np.array([float(row["airspeed_m_s"]) for row in rows]),
        "pitch": np.deg2rad([float(row["pitch_deg"]) for row in rows]),
        "lift": np.array([float(row["collective_lift"]) for row in rows]),
        "pusher": np.array([float(row["pusher"]) for row in rows]),
        "elevator": np.deg2rad([float(row["elevator_deg"]) for row in rows]),
    }


def speed_reference(time_seconds: float, target: float) -> float:
    """Conservative reference: hover 2 s, then accelerate at 1 m/s/s."""
    return float(np.clip(time_seconds - 2.0, 0.0, target))


def references(
    initial_x: float,
    altitude: float,
    time_seconds: float,
    controller: StandardVtolNmpc,
    corridor: dict[str, np.ndarray],
    target_speed: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Construct horizon references and scheduled elevator parameters."""
    x_ref = np.zeros((controller.N + 1, 10))
    u_ref = np.zeros((controller.N, 5))
    params = np.zeros((controller.N + 1, controller.model.parameter_size))
    params[:, 4] = 1.0
    forward_position = initial_x
    for stage in range(controller.N + 1):
        stage_time = time_seconds + stage * controller.dt
        speed = speed_reference(stage_time, target_speed)
        pitch = np.interp(speed, corridor["speed"], corridor["pitch"])
        lift = np.interp(speed, corridor["speed"], corridor["lift"])
        pusher = np.interp(speed, corridor["speed"], corridor["pusher"])
        elevator = np.interp(speed, corridor["speed"], corridor["elevator"])
        if stage:
            forward_position += speed * controller.dt
        x_ref[stage] = [
            forward_position, 0.0, altitude, speed, 0.0, 0.0,
            np.cos(0.5 * pitch), 0.0, np.sin(0.5 * pitch), 0.0,
        ]
        params[stage, 3] = elevator
        if stage < controller.N:
            u_ref[stage, 0:2] = [lift, pusher]
    return x_ref, u_ref, params


def rk4_step(model, state, control, parameters, dt):
    """Integrate the independent NumPy prediction plant by one sample."""
    wind = parameters[:3]
    elevator = parameters[3]
    lift_weight = parameters[4]
    dynamics = lambda value: model.derivative(
        value, control, wind, elevator, lift_weight
    )
    k1 = dynamics(state)
    k2 = dynamics(state + 0.5 * dt * k1)
    k3 = dynamics(state + 0.5 * dt * k2)
    k4 = dynamics(state + dt * k3)
    result = state + dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
    result[6:10] /= np.linalg.norm(result[6:10])
    return result


def limit_command(previous, requested, dt):
    """Apply the same command-slew safety layer intended for the ROS node."""
    rates = np.array([0.12, 0.15, 0.8, 0.8, 0.6])
    return previous + np.clip(requested - previous, -rates * dt, rates * dt)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-speed", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--output", type=Path, default=Path("results/standard_vtol_nmpc"))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    corridor = load_corridor(root / "results/standard_vtol_trim_corridor/trim_corridor.csv")
    controller = StandardVtolNmpc(build_directory=root / "build/standard_vtol_nmpc")
    plant = StandardVtolTransitionRateModel()
    state = plant.hover_state()
    state[2] = 20.0
    command = plant.hover_control()
    count = int(round(args.duration / controller.dt))
    states = [state.copy()]
    controls = []
    timings = []
    failures = 0

    for index in range(count):
        time_seconds = index * controller.dt
        x_ref, u_ref, params = references(
            state[0], 20.0, time_seconds, controller, corridor, args.target_speed
        )
        solution = controller.solve(state, x_ref, u_ref, params)
        if solution.status != 0:
            failures += 1
            requested = command
        else:
            requested = solution.control
        command = limit_command(command, requested, controller.dt)
        state = rk4_step(plant, state, command, params[0], controller.dt)
        states.append(state.copy())
        controls.append(command.copy())
        timings.append(solution.solve_time)

    states = np.asarray(states)
    controls = np.asarray(controls)
    timings = np.asarray(timings)
    args.output.mkdir(parents=True, exist_ok=True)
    np.savez(args.output / "offline_transition.npz", states=states, controls=controls, timings=timings)
    time_axis = np.arange(len(states)) * controller.dt
    figure, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    axes[0].plot(time_axis, states[:, 3], label="forward speed")
    axes[0].plot(time_axis, [speed_reference(t, args.target_speed) for t in time_axis], "--", label="reference")
    axes[0].set_ylabel("m/s")
    axes[0].legend()
    axes[1].plot(time_axis, states[:, 2] - 20.0)
    axes[1].set_ylabel("altitude error [m]")
    axes[2].plot(time_axis[:-1], controls[:, 0], label="lift")
    axes[2].plot(time_axis[:-1], controls[:, 1], label="pusher")
    axes[2].set_ylabel("normalized command")
    axes[2].set_xlabel("time [s]")
    axes[2].legend()
    figure.tight_layout()
    figure.savefig(args.output / "offline_transition.png", dpi=160)
    print(f"solver_failures={failures}")
    print(f"max_altitude_error_m={np.max(np.abs(states[:, 2] - 20.0)):.3f}")
    print(f"final_speed_m_s={states[-1, 3]:.3f}")
    print(f"solve_time_p99_ms={1000.0 * np.percentile(timings, 99):.3f}")


if __name__ == "__main__":
    main()
