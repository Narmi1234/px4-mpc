#!/usr/bin/env python3
"""Offline closed-loop gate for the 16-state robust transition NMPC."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from px4_mpc.controllers.standard_vtol_robust_nmpc import (
    StandardVtolRobustNmpc,
)


def load_corridor(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    return {
        "speed": np.array([float(row["airspeed_m_s"]) for row in rows]),
        "pitch": np.deg2rad([float(row["pitch_deg"]) for row in rows]),
        "lift": np.array([float(row["collective_lift"]) for row in rows]),
        "pusher": np.array([float(row["pusher"]) for row in rows]),
        "elevator": np.deg2rad([float(row["elevator_deg"]) for row in rows]),
    }


def speed_reference(time_seconds: float, target: float, acceleration: float) -> float:
    return float(np.clip((time_seconds - 2.0) * acceleration, 0.0, target))


def scheduled_lambda(speed: float, target_speed: float) -> float:
    """Conservative lift unloading: retain MC support through 7 m/s."""
    scale = target_speed / 15.0
    nodes = scale * np.array([0.0, 5.0, 7.0, 9.0, 12.0, 15.0])
    return float(np.interp(speed, nodes, [1.0, 1.0, 0.85, 0.60, 0.25, 0.0]))


def scheduled_pitch(speed: float, target_speed: float) -> float:
    """Nose-down transition corridor without the low-speed trim singularity."""
    scale = target_speed / 15.0
    nodes = scale * np.array([0.0, 4.0, 7.0, 9.0, 12.0, 15.0])
    degrees = np.array([0.0, -3.0, -6.0, -8.0, -4.0, -1.36])
    return float(np.deg2rad(np.interp(speed, nodes, degrees)))


def references(
    state: np.ndarray,
    altitude: float,
    time_seconds: float,
    controller: StandardVtolRobustNmpc,
    corridor: dict[str, np.ndarray],
    target_speed: float,
    acceleration: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    x_ref = np.zeros((controller.N + 1, controller.model.state_size))
    u_ref = np.zeros((controller.N, controller.model.control_size))
    parameters = np.zeros((controller.N + 1, controller.model.parameter_size))
    forward_position = float(state[0])
    hover = controller.model.plant.hover_command

    for stage in range(controller.N + 1):
        stage_time = time_seconds + stage * controller.dt
        speed = speed_reference(stage_time, target_speed, acceleration)
        pitch = scheduled_pitch(speed, target_speed)
        lift_fraction = scheduled_lambda(speed, target_speed)
        corridor_lift = float(np.interp(speed, corridor["speed"], corridor["lift"]))
        pusher = float(np.interp(speed, corridor["speed"], corridor["pusher"]))
        elevator = float(np.interp(speed, corridor["speed"], corridor["elevator"]))
        if stage:
            forward_position += speed * controller.dt
        x_ref[stage, 0:6] = [
            forward_position, 0.0, altitude, speed, 0.0, 0.0
        ]
        x_ref[stage, 6:10] = [
            np.cos(0.5 * pitch), 0.0, np.sin(0.5 * pitch), 0.0
        ]
        x_ref[stage, 13:16] = [0.0, 0.0, (1.0 - lift_fraction) * elevator]

        if stage < controller.N:
            if lift_fraction > 0.03:
                collective = np.clip(corridor_lift / lift_fraction, 0.0, 0.68)
            else:
                collective = hover
            u_ref[stage] = [
                collective, pusher, 0.0, 0.0, 0.0, lift_fraction
            ]
    return x_ref, u_ref, parameters


def rk4_step(function, state, control, parameters, dt):
    derivative = lambda value: np.asarray(
        function(value, control, parameters), dtype=float
    ).reshape(-1)
    k1 = derivative(state)
    k2 = derivative(state + 0.5 * dt * k1)
    k3 = derivative(state + 0.5 * dt * k2)
    k4 = derivative(state + dt * k3)
    result = state + dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
    result[6:10] /= np.linalg.norm(result[6:10])
    return result


def limit_command(previous, requested, dt):
    # The live interface must apply the same actuator-safe slew layer.
    rates = np.array([0.20, 0.18, 0.8, 0.8, 0.6, 0.08])
    limited = previous + np.clip(requested - previous, -rates * dt, rates * dt)
    return np.clip(
        limited,
        [0.0, 0.0, -0.45, -0.45, -0.30, 0.0],
        [0.70, 0.70, 0.45, 0.45, 0.30, 1.0],
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-speed", type=float, default=15.0)
    parser.add_argument("--acceleration", type=float, default=0.60)
    parser.add_argument("--duration", type=float, default=34.0)
    parser.add_argument("--horizon-steps", type=int, default=30)
    parser.add_argument("--horizon-seconds", type=float, default=2.0)
    parser.add_argument("--pitch-disturbance", type=float, default=0.0)
    parser.add_argument(
        "--unmodeled-disturbance",
        action="store_true",
        help="keep the OCP disturbance estimate at zero",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("results/standard_vtol_robust_transition/nominal"),
    )
    parser.add_argument(
        "--progress",
        action="store_true",
        help="print closed-loop simulation progress while the OCP is running",
    )
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    corridor = load_corridor(
        root / "results/standard_vtol_trim_corridor/trim_corridor.csv"
    )
    build_name = (
        f"standard_vtol_robust_nmpc_n{args.horizon_steps}_"
        f"tf{int(round(1000.0 * args.horizon_seconds))}ms"
    )
    controller = StandardVtolRobustNmpc(
        horizon_steps=args.horizon_steps,
        horizon_seconds=args.horizon_seconds,
        build_directory=root / "build" / build_name,
    )
    function = controller.model.function()
    state = controller.model.hover_state()
    state[2] = 30.0
    command = controller.model.hover_control()
    count = int(round(args.duration / controller.dt))
    states = [state.copy()]
    controls = []
    timings = []
    statuses = []

    disturbance_amplitude = np.clip(
        args.pitch_disturbance,
        -controller.model.pitch_disturbance_bound,
        controller.model.pitch_disturbance_bound,
    )
    for index in range(count):
        time_seconds = index * controller.dt
        if args.progress and index % max(1, int(round(5.0 / controller.dt))) == 0:
            print(
                f"  progress: simulated {time_seconds:.0f}/{args.duration:.0f} s "
                f"({index}/{count} NMPC solves)",
                flush=True,
            )
        x_ref, u_ref, prediction_parameters = references(
            state,
            30.0,
            time_seconds,
            controller,
            corridor,
            args.target_speed,
            args.acceleration,
        )
        if not args.unmodeled_disturbance:
            prediction_parameters[:-1, 5] = disturbance_amplitude * (
                4.0 * u_ref[:, 5] * (1.0 - u_ref[:, 5])
            )
            prediction_parameters[-1, 5] = prediction_parameters[-2, 5]
        solution = controller.solve(
            state, x_ref, u_ref, prediction_parameters
        )
        statuses.append(solution.status)
        requested = command if solution.status != 0 else solution.control
        command = limit_command(command, requested, controller.dt)
        plant_parameters = np.zeros(controller.model.parameter_size)
        scheduled_blend = u_ref[0, 5]
        plant_parameters[5] = disturbance_amplitude * (
            4.0 * scheduled_blend * (1.0 - scheduled_blend)
        )
        state = rk4_step(
            function, state, command, plant_parameters, controller.dt
        )
        states.append(state.copy())
        controls.append(command.copy())
        timings.append(solution.solve_time)

    if args.progress:
        print(
            f"  progress: simulated {args.duration:.0f}/{args.duration:.0f} s "
            f"({count}/{count} NMPC solves)",
            flush=True,
        )

    states = np.asarray(states)
    controls = np.asarray(controls)
    timings = np.asarray(timings)
    statuses = np.asarray(statuses)
    time_axis = np.arange(len(states)) * controller.dt
    pitch = np.rad2deg(
        np.arcsin(np.clip(2.0 * (
            states[:, 6] * states[:, 8] - states[:, 9] * states[:, 7]
        ), -1.0, 1.0))
    )
    metrics = {
        "target_speed_m_s": args.target_speed,
        "horizon_steps": args.horizon_steps,
        "horizon_seconds": args.horizon_seconds,
        "pitch_disturbance_amplitude_frd_rad_s2": float(disturbance_amplitude),
        "disturbance_estimated_by_ocp": not args.unmodeled_disturbance,
        "solver_failures": int(np.count_nonzero(statuses)),
        "max_altitude_error_m": float(np.max(np.abs(states[:, 2] - 30.0))),
        "max_vertical_speed_m_s": float(np.max(np.abs(states[:, 5]))),
        "max_abs_pitch_deg": float(np.max(np.abs(pitch))),
        "final_forward_speed_m_s": float(states[-1, 3]),
        "final_lambda": float(controls[-1, 5]),
        "solve_time_p99_ms": float(1000.0 * np.percentile(timings, 99)),
    }
    passed = (
        metrics["solver_failures"] == 0
        and metrics["max_altitude_error_m"] <= 2.0
        and metrics["max_vertical_speed_m_s"] <= 2.0
        and metrics["max_abs_pitch_deg"] <= 22.0
        and metrics["final_forward_speed_m_s"] >= 0.85 * args.target_speed
        and metrics["final_lambda"] <= 0.10
    )
    metrics["status"] = "PASS" if passed else "FAIL"

    args.output.mkdir(parents=True, exist_ok=True)
    np.savez(
        args.output / "closed_loop.npz",
        time=time_axis,
        states=states,
        controls=controls,
        solve_time=timings,
        solver_status=statuses,
    )
    (args.output / "metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    for key, value in metrics.items():
        print(f"{key}={value}")
    print(f"robust_transition_offline={metrics['status']}")
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
