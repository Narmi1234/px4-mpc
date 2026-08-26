#!/usr/bin/env python3
"""Offline closed-loop rehearsal of Gate A: 3 m/s MC pusher feedback."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from px4_mpc.controllers.standard_vtol_nmpc import StandardVtolNmpc
from px4_mpc.controllers.standard_vtol_output import (
    govern_pusher_forward_envelope,
    govern_pusher_forward_lateral,
    limit_pusher_forward_command,
    vertical_hover_lift,
)
from px4_mpc.models.mc_forward_profile import (
    McForwardProfile,
    pusher_forward_feedforward,
    pusher_forward_speed_reference_state,
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


def quaternion_product(left, right):
    """Return scalar-first Hamilton product used for attitude disturbances."""
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.array(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ]
    )


def references(
    controller,
    profile,
    hold_state,
    current_state,
    profile_time,
    pusher_limit,
):
    """Build the exact Gate A horizon for yaw zero."""
    direction = np.array([1.0, 0.0])
    samples = [
        profile.sample(profile_time + stage * controller.dt)
        for stage in range(controller.N + 1)
    ]
    base_sample = samples[0]
    x_ref = np.vstack(
        [
            pusher_forward_speed_reference_state(
                hold_state,
                current_state,
                direction,
                base_sample,
                sample,
            )
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
            command_limit=pusher_limit,
        )
        for sample in samples[:-1]
    ]
    parameters = np.zeros((controller.N + 1, 4))
    return x_ref, u_ref, parameters


def main() -> None:
    """Run the Gate A rehearsal and enforce its live acceptance envelope."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=26.5)
    parser.add_argument("--target-speed", type=float, default=3.0)
    parser.add_argument("--acceleration", type=float, default=0.50)
    parser.add_argument("--hold-seconds", type=float, default=2.0)
    parser.add_argument("--start-delay-seconds", type=float, default=2.0)
    parser.add_argument("--pusher-limit", type=float, default=0.10)
    parser.add_argument("--minimum-peak-speed", type=float, default=2.5)
    parser.add_argument("--maximum-speed", type=float, default=3.5)
    parser.add_argument("--maximum-final-speed", type=float, default=0.35)
    parser.add_argument("--maximum-altitude-error", type=float, default=0.30)
    parser.add_argument("--maximum-cross-track", type=float, default=1.0)
    parser.add_argument("--maximum-tilt-degrees", type=float, default=10.0)
    parser.add_argument("--initial-lateral-speed", type=float, default=0.0)
    parser.add_argument("--initial-forward-speed", type=float, default=0.0)
    parser.add_argument("--forward-gust-speed", type=float, default=0.0)
    parser.add_argument("--forward-gust-time", type=float, default=7.0)
    parser.add_argument("--lateral-gust-speed", type=float, default=0.0)
    parser.add_argument("--lateral-gust-time", type=float, default=15.0)
    parser.add_argument("--pitch-gust-degrees", type=float, default=0.0)
    parser.add_argument("--pitch-gust-time", type=float, default=10.0)
    parser.add_argument(
        "--rate-delay-seconds",
        type=float,
        default=0.075,
        help="Measured PX4 body-rate tracking delay used by the rehearsal",
    )
    parser.add_argument(
        "--pusher-effectiveness",
        type=float,
        default=0.84,
        help="ULog-fitted pusher thrust scale applied only to the rehearsal plant",
    )
    parser.add_argument(
        "--rate-effectiveness",
        type=float,
        default=1.0,
        help="Stress scale applied to body rates in the rehearsal plant",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("results/standard_vtol_pusher_forward_offline"),
    )
    arguments = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    controller = StandardVtolNmpc(
        build_directory=root / (
            "build/standard_vtol_nmpc_offline_pusher_"
            f"{round(1000.0 * arguments.pusher_limit):03d}"
        ),
        control_lower_bounds=np.array([0.0, 0.0, -0.50, -0.50, -0.30]),
        control_upper_bounds=np.array(
            [0.65, arguments.pusher_limit, 0.50, 0.50, 0.30]
        ),
    )
    plant = StandardVtolTransitionRateModel(controller.model.plant)
    profile = McForwardProfile(
        target_speed=arguments.target_speed,
        acceleration=arguments.acceleration,
        hold_seconds=arguments.hold_seconds,
        start_delay_seconds=arguments.start_delay_seconds,
    )
    state = plant.hover_state()
    state[2] = 20.0
    state[3] = arguments.initial_forward_speed
    hold_state = state.copy()
    hold_state[3:6] = 0.0
    state[4] = arguments.initial_lateral_speed
    command = plant.hover_control()
    states = [state.copy()]
    controls = []
    applied_controls = []
    raw_controls = []
    state_references = []
    solve_times = []
    solver_failures = 0
    consecutive_failures = 0
    rate_delay_steps = max(
        0, int(np.ceil(arguments.rate_delay_seconds / controller.dt))
    )
    rate_history = [command[2:5].copy()] * rate_delay_steps

    for index in range(round(arguments.duration / controller.dt)):
        elapsed = index * controller.dt
        if (
            arguments.forward_gust_speed != 0.0
            and index == round(arguments.forward_gust_time / controller.dt)
        ):
            state[3] += arguments.forward_gust_speed
        if (
            arguments.lateral_gust_speed != 0.0
            and index == round(arguments.lateral_gust_time / controller.dt)
        ):
            state[4] += arguments.lateral_gust_speed
        if (
            arguments.pitch_gust_degrees != 0.0
            and index == round(arguments.pitch_gust_time / controller.dt)
        ):
            angle = np.deg2rad(arguments.pitch_gust_degrees)
            disturbance = np.array(
                [np.cos(0.5 * angle), 0.0, np.sin(0.5 * angle), 0.0]
            )
            state[6:10] = quaternion_product(
                state[6:10], disturbance
            )
            state[6:10] /= np.linalg.norm(state[6:10])
        profile_time = max(0.0, elapsed - 0.5)
        x_ref, u_ref, parameters = references(
            controller,
            profile,
            hold_state,
            state,
            profile_time,
            arguments.pusher_limit,
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
            previous_command = command.copy()
            command = limit_pusher_forward_command(
                previous_command,
                requested,
                controller.dt,
                pusher_limit=arguments.pusher_limit,
            )
            roll = np.arctan2(
                2.0 * (state[6] * state[7] + state[8] * state[9]),
                1.0 - 2.0 * (state[7] ** 2 + state[8] ** 2),
            )
            command = govern_pusher_forward_lateral(
                previous_command,
                command,
                state[1] - hold_state[1],
                state[4],
                roll,
                controller.dt,
            )
            sample = profile.sample(profile_time)
            pitch = np.arcsin(
                np.clip(
                    2.0
                    * (state[6] * state[8] - state[9] * state[7]),
                    -1.0,
                    1.0,
                )
            )
            command = govern_pusher_forward_envelope(
                previous_command,
                command,
                state[3],
                sample.speed,
                profile.target_speed,
                pitch,
                controller.dt,
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
        rate_history.append(command[2:5].copy())
        applied_command = command.copy()
        applied_command[1] *= np.sqrt(arguments.pusher_effectiveness)
        applied_command[2:5] = (
            arguments.rate_effectiveness * rate_history.pop(0)
        )
        state = rk4_step(
            plant, state, applied_command, parameters[0], controller.dt
        )
        states.append(state.copy())
        controls.append(command.copy())
        applied_controls.append(applied_command.copy())
        state_references.append(x_ref[0].copy())

    states = np.asarray(states)
    controls = np.asarray(controls)
    raw_controls = np.asarray(raw_controls)
    applied_controls = np.asarray(applied_controls)
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
        "max_horizontal_speed_m_s": float(
            np.max(np.linalg.norm(states[:, 3:5], axis=1))
        ),
        "final_speed_m_s": float(np.linalg.norm(states[-1, 3:5])),
        "final_position_error_m": float(
            np.linalg.norm(states[-1, 0:2] - state_references[-1, 0:2])
        ),
        "max_tracking_error_m": float(
            np.max(np.linalg.norm(position_error, axis=1))
        ),
        "max_cross_track_m": float(np.max(np.abs(states[:, 1] - hold_state[1]))),
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
        "rate_delay_seconds": rate_delay_steps * controller.dt,
        "pusher_effectiveness": arguments.pusher_effectiveness,
        "rate_effectiveness": arguments.rate_effectiveness,
    }
    passed = (
        len(controls) == round(arguments.duration / controller.dt)
        and metrics["solver_failures"] == 0
        and arguments.minimum_peak_speed
        <= metrics["max_speed_m_s"]
        <= arguments.maximum_speed
        and metrics["max_horizontal_speed_m_s"] <= arguments.maximum_speed
        and metrics["final_speed_m_s"] <= arguments.maximum_final_speed
        and metrics["final_position_error_m"] <= 1.0
        and metrics["max_tracking_error_m"] <= 2.0
        and metrics["max_cross_track_m"] <= arguments.maximum_cross_track
        and metrics["max_altitude_error_m"] <= arguments.maximum_altitude_error
        and metrics["max_vertical_speed_m_s"] <= 0.75
        and metrics["max_tilt_deg"] <= arguments.maximum_tilt_degrees
        and 0.05
        <= metrics["max_pusher"]
        <= arguments.pusher_limit + 1.0e-12
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
