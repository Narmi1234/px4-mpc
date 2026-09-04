#!/usr/bin/env python3
"""Closed-loop check of the exact guarded L2/L3 reference and output layer."""

import argparse
import math

from types import MethodType, SimpleNamespace

import numpy as np

from px4_mpc.controllers.standard_vtol_output import vertical_hover_lift
from px4_mpc.controllers.standard_vtol_robust_nmpc import (
    StandardVtolRobustNmpc,
)
from px4_mpc.standard_vtol_robust_shadow_node import (
    StandardVtolRobustShadow,
)
from simulate_standard_vtol_robust_transition import rk4_step


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--level", type=int, choices=(2, 3), default=2)
    parser.add_argument("--target-speed", type=float)
    parser.add_argument("--acceleration", type=float)
    parser.add_argument("--brake-rate", type=float, default=0.5)
    parser.add_argument("--recovery-seconds", type=float, default=0.0)
    parser.add_argument("--brake-entry-lambda", type=float, default=0.7)
    parser.add_argument("--minimum-lambda", type=float)
    parser.add_argument("--pusher-max", type=float)
    parser.add_argument("--collective-min", type=float, default=0.30)
    parser.add_argument("--effective-lift-min", type=float, default=0.0)
    parser.add_argument("--pitch-rate-limit", type=float, default=0.18)
    parser.add_argument("--pitch-damping-gain", type=float, default=0.0)
    parser.add_argument("--vertical-correction-gain", type=float, default=1.0)
    parser.add_argument("--inject-time", type=float, default=-1.0)
    parser.add_argument("--inject-cross-track", type=float, default=0.0)
    parser.add_argument("--inject-lateral-speed", type=float, default=0.0)
    parser.add_argument("--inject-yaw-deg", type=float, default=0.0)
    args = parser.parse_args()
    level = args.level
    target_speed = args.target_speed or (12.0 if level == 3 else 9.0)
    acceleration = args.acceleration or (0.35 if level == 3 else 0.4)
    commanded_minimum_lambda = (
        args.minimum_lambda
        if args.minimum_lambda is not None
        else 0.2 if level == 3 else 0.5
    )
    configured_pusher_max = args.pusher_max or (
        0.50 if level == 3 else 0.35
    )
    controller = StandardVtolRobustNmpc(
        horizon_steps=20,
        horizon_seconds=2.0,
        build_directory="build/standard_vtol_robust_nmpc_n20_tf2000ms",
    )
    gate = SimpleNamespace(
        test_mode=f"allocation_l{level}",
        controller=controller,
        l1_target_speed=5.0,
        l1_acceleration=0.4,
        l1_hold_seconds=3.0,
        l1_min_lambda=0.8,
        l1_pusher_max=0.25,
        l2_target_speed=9.0,
        l2_acceleration=0.4,
        l2_hold_seconds=3.0,
        l2_min_lambda=0.5,
        l2_pusher_max=0.35,
        l3_target_speed=target_speed,
        l3_acceleration=acceleration,
        l3_brake_rate=args.brake_rate,
        l3_recovery_seconds=args.recovery_seconds,
        l3_brake_entry_lambda=args.brake_entry_lambda,
        l3_hold_seconds=4.0,
        l3_min_lambda=commanded_minimum_lambda,
        l3_pusher_max=configured_pusher_max,
        l3_collective_min=args.collective_min,
        l3_effective_lift_min=args.effective_lift_min,
        l3_pitch_rate_limit=args.pitch_rate_limit,
        l3_pitch_damping_gain=args.pitch_damping_gain,
        l3_vertical_correction_gain=args.vertical_correction_gain,
        reference_forward=np.array([1.0, 0.0]),
        reference_lateral=np.array([0.0, 1.0]),
    )
    for name in (
        "_allocation_configuration",
        "_l1_speed_reference",
        "_l1_total_seconds",
        "_l1_references",
        "_allocation_control_bounds",
        "_forward_speed",
        "_cross_track",
    ):
        setattr(
            gate,
            name,
            MethodType(getattr(StandardVtolRobustShadow, name), gate),
        )
    gate._yaw = StandardVtolRobustShadow._yaw
    gate._yaw_pitch_quaternion = (
        StandardVtolRobustShadow._yaw_pitch_quaternion
    )
    gate._l2_pitch_reference = StandardVtolRobustShadow._l2_pitch_reference

    state = controller.model.hover_state()
    state[2] = 30.0
    gate.state = state
    gate.reference = state.copy()
    command = controller.model.hover_control()
    dynamics = controller.model.function()
    dt = 0.05
    maxima = np.zeros(6)
    maximum_times = np.zeros(6)
    maximum_signed_altitude = 0.0
    maximum_altitude_control = command.copy()
    minimum_lambda = 1.0
    solver_failures = 0
    disturbance_injected = False
    count = int(np.ceil(gate._l1_total_seconds() / dt))

    for index in range(count):
        elapsed = index * dt
        if (
            not disturbance_injected
            and args.inject_time >= 0.0
            and elapsed >= args.inject_time
        ):
            state[1] += args.inject_cross_track
            state[4] += args.inject_lateral_speed
            angle = math.radians(args.inject_yaw_deg)
            yaw_delta = np.array(
                [math.cos(0.5 * angle), 0.0, 0.0, math.sin(0.5 * angle)]
            )
            w1, x1, y1, z1 = yaw_delta
            w2, x2, y2, z2 = state[6:10]
            state[6:10] = np.array(
                [
                    w1*w2 - x1*x2 - y1*y2 - z1*z2,
                    w1*x2 + x1*w2 + y1*z2 - z1*y2,
                    w1*y2 - x1*z2 + y1*w2 + z1*x2,
                    w1*z2 + x1*y2 - y1*x2 + z1*w2,
                ]
            )
            state[6:10] /= np.linalg.norm(state[6:10])
            disturbance_injected = True
        gate.state = state
        x_ref, u_ref, parameters = gate._l1_references(elapsed)
        lower_bounds, upper_bounds = gate._allocation_control_bounds(u_ref)
        solution = controller.solve(
            state,
            x_ref,
            u_ref,
            parameters,
            control_lower_bounds=lower_bounds,
            control_upper_bounds=upper_bounds,
        )
        solver_failures += int(solution.status != 0)
        if solution.status != 0:
            print(f"first_solver_failure_time_s={elapsed}")
            print(f"first_solver_failure_status={solution.status}")
            print(f"first_solver_failure_speed_m_s={state[3]}")
            print(f"first_solver_failure_lambda={command[5]}")
            print(f"first_solver_failure_reference_m_s={gate._l1_speed_reference(elapsed)}")
            break
        requested = (
            solution.control.copy()
            if solution.status == 0
            else command.copy()
        )
        minimum_command_lambda = commanded_minimum_lambda
        pusher_max = configured_pusher_max
        requested[5] = np.clip(
            requested[5], minimum_command_lambda, 1.0
        )
        base_lift = vertical_hover_lift(
            controller.model.plant,
            state[2] - 30.0,
            state[5],
        )
        correction = args.vertical_correction_gain * (
            base_lift - controller.model.plant.hover_command
        ) / requested[5]
        collective_minimum = args.collective_min if level == 3 else 0.30
        if level == 3:
            collective_minimum = (
                StandardVtolRobustShadow._minimum_collective_for_allocation(
                    args.collective_min,
                    args.effective_lift_min,
                    requested[5],
                    state[3],
                )
            )
        requested[0] = np.clip(
            requested[0] + correction,
            collective_minimum,
            0.70,
        )
        requested[1] = np.clip(requested[1], 0.0, pusher_max)
        requested[2:5] = np.clip(
            np.array([0.40, 1.00, 0.40]) * requested[2:5],
            [-0.12, -args.pitch_rate_limit, -0.10],
            [0.12, args.pitch_rate_limit, 0.10],
        )
        if level == 3:
            requested[3] = StandardVtolRobustShadow._damped_pitch_rate_command(
                requested[3], state[11], requested[5],
                args.pitch_damping_gain, args.pitch_rate_limit,
            )
        slew = np.array([0.10, 0.05, 0.20, 0.20, 0.15, 0.05])
        limited = command + np.clip(
            requested - command, -slew * dt, slew * dt
        )
        if state[3] > gate._l1_speed_reference(elapsed) + 0.40:
            limited[1] = max(0.0, command[1] - 0.15 * dt)
        command = limited
        minimum_lambda = min(minimum_lambda, float(command[5]))
        state = rk4_step(
            dynamics,
            state,
            command,
            controller.model.nominal_parameters(),
            dt,
        )
        pitch = np.arcsin(
            np.clip(
                2.0 * (state[6] * state[8] - state[9] * state[7]),
                -1.0,
                1.0,
            )
        )
        sample_metrics = np.asarray(
            [
                state[3],
                abs(state[2] - 30.0),
                abs(state[5]),
                abs(pitch),
                command[1],
                abs(state[1]),
            ]
        )
        improved = sample_metrics > maxima
        maximum_times[improved] = elapsed
        maxima = np.maximum(maxima, sample_metrics)
        if improved[1]:
            maximum_signed_altitude = float(state[2] - 30.0)
            maximum_altitude_control = command.copy()

    metrics = {
        "solver_failures": solver_failures,
        "max_forward_speed_m_s": maxima[0],
        "max_altitude_error_m": maxima[1],
        "max_vertical_speed_m_s": maxima[2],
        "max_abs_pitch_deg": np.rad2deg(maxima[3]),
        "max_pusher": maxima[4],
        "minimum_lambda": minimum_lambda,
        "max_cross_track_m": maxima[5],
        "max_altitude_error_time_s": maximum_times[1],
        "max_altitude_error_signed_m": maximum_signed_altitude,
        "max_altitude_error_control": np.round(
            maximum_altitude_control, 4
        ).tolist(),
        "final_forward_speed_m_s": state[3],
        "final_lambda": command[5],
    }
    minimum_speed = target_speed - 1.0
    altitude_limit = 1.2 if level == 3 else 1.0
    vertical_speed_limit = 0.9 if level == 3 else 0.8
    tilt_limit = 18.0 if level == 3 else 15.0
    lambda_proof = commanded_minimum_lambda + 0.10
    settle_speed = 0.7 if level == 3 else 0.5
    passed = (
        solver_failures == 0
        and maxima[0] >= minimum_speed
        and maxima[1] <= altitude_limit
        and maxima[2] <= vertical_speed_limit
        and np.rad2deg(maxima[3]) <= tilt_limit
        and maxima[5] <= (2.5 if level == 3 else 2.0)
        and minimum_lambda <= lambda_proof
        and abs(state[3]) <= settle_speed
        and command[5] >= 0.95
    )
    for key, value in metrics.items():
        print(f"{key}={value}")
    result_name = f"ROBUST_ALLOCATION_L{level}_LIVE_LAYER"
    print(f"{result_name}={'PASS' if passed else 'FAIL'}")
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
