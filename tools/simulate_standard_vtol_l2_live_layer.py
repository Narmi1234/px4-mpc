#!/usr/bin/env python3
"""Closed-loop check of the exact guarded L2 reference and output layer."""

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
    controller = StandardVtolRobustNmpc(
        horizon_steps=20,
        horizon_seconds=2.0,
        build_directory="build/standard_vtol_robust_nmpc_n20_tf2000ms",
    )
    gate = SimpleNamespace(
        test_mode="allocation_l2",
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
    maxima = np.zeros(5)
    minimum_lambda = 1.0
    solver_failures = 0
    count = int(np.ceil(gate._l1_total_seconds() / dt))

    for index in range(count):
        elapsed = index * dt
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
        requested = (
            solution.control.copy()
            if solution.status == 0
            else command.copy()
        )
        requested[5] = np.clip(requested[5], 0.5, 1.0)
        base_lift = vertical_hover_lift(
            controller.model.plant,
            state[2] - 30.0,
            state[5],
        )
        correction = (
            base_lift - controller.model.plant.hover_command
        ) / requested[5]
        requested[0] = np.clip(requested[0] + correction, 0.30, 0.70)
        requested[1] = np.clip(requested[1], 0.0, 0.35)
        requested[2:5] = np.clip(
            np.array([0.40, 1.00, 0.40]) * requested[2:5],
            [-0.12, -0.18, -0.10],
            [0.12, 0.18, 0.10],
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
            np.zeros(controller.model.parameter_size),
            dt,
        )
        pitch = np.arcsin(
            np.clip(
                2.0 * (state[6] * state[8] - state[9] * state[7]),
                -1.0,
                1.0,
            )
        )
        maxima = np.maximum(
            maxima,
            [
                state[3],
                abs(state[2] - 30.0),
                abs(state[5]),
                abs(pitch),
                command[1],
            ],
        )

    metrics = {
        "solver_failures": solver_failures,
        "max_forward_speed_m_s": maxima[0],
        "max_altitude_error_m": maxima[1],
        "max_vertical_speed_m_s": maxima[2],
        "max_abs_pitch_deg": np.rad2deg(maxima[3]),
        "max_pusher": maxima[4],
        "minimum_lambda": minimum_lambda,
        "final_forward_speed_m_s": state[3],
        "final_lambda": command[5],
    }
    passed = (
        solver_failures == 0
        and maxima[0] >= 8.0
        and maxima[1] <= 1.0
        and maxima[2] <= 0.8
        and np.rad2deg(maxima[3]) <= 15.0
        and minimum_lambda <= 0.60
        and abs(state[3]) <= 0.5
        and command[5] >= 0.95
    )
    for key, value in metrics.items():
        print(f"{key}={value}")
    print(f"ROBUST_ALLOCATION_L2_LIVE_LAYER={'PASS' if passed else 'FAIL'}")
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
