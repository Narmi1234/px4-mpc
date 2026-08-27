#!/usr/bin/env python3
"""Offline closed-loop rehearsal of the first NMPC front/back transition."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from px4_mpc.controllers.standard_vtol_nmpc import StandardVtolNmpc
from px4_mpc.controllers.standard_vtol_output import (
    govern_pusher_forward_lateral,
    govern_transition_pitch,
    govern_transition_speed,
    limit_transition_command,
    pretransition_lift_command,
    vertical_hover_lift,
)
from px4_mpc.models.mc_forward_profile import (
    pusher_forward_feedforward,
    pusher_forward_speed_reference_state,
)
from px4_mpc.models.standard_vtol_gz_model import StandardVtolTransitionRateModel
from px4_mpc.models.transition_gate_d import (
    GateDStateMachine,
    VTOL_FW,
    VTOL_MC,
    VTOL_TRANSITION_TO_FW,
    VTOL_TRANSITION_TO_MC,
    px4_mc_weight,
    transition_pitch_and_elevator,
    transition_pusher_trim,
)


def rk4_step(model, state, control, parameters, dt):
    def dynamics(value):
        return model.derivative(
            value, control, parameters[:3], parameters[3], parameters[4]
        )

    k1 = dynamics(state)
    k2 = dynamics(state + 0.5 * dt * k1)
    k3 = dynamics(state + 0.5 * dt * k2)
    k4 = dynamics(state + dt * k3)
    result = state + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
    result[6:10] /= np.linalg.norm(result[6:10])
    return result


def yaw_pitch_quaternion(pitch):
    return np.array([math.cos(0.5 * pitch), 0.0, math.sin(0.5 * pitch), 0.0])


def references(controller, gate, hold, state, elapsed, vtol_state):
    direction = np.array([1.0, 0.0])
    airspeed = max(0.0, float(state[3]))
    phase_seconds = max(0.0, elapsed - gate.entered_s)
    weight = px4_mc_weight(vtol_state, airspeed, phase_seconds)
    samples = [
        gate.sample(elapsed + stage * controller.dt)
        for stage in range(controller.N + 1)
    ]
    x_ref = []
    elevators = []
    for sample in samples:
        reference = pusher_forward_speed_reference_state(
            hold, state, direction, samples[0], sample
        )
        pitch, elevator = transition_pitch_and_elevator(
            sample.speed, weight, vtol_state
        )
        reference[6:10] = yaw_pitch_quaternion(pitch)
        x_ref.append(reference)
        elevators.append(elevator)
    u_ref = np.zeros((controller.N, 5))
    u_ref[:, 0] = controller.model.plant.hover_command
    for stage, sample in enumerate(samples[:-1]):
        mc = pusher_forward_feedforward(
            controller.model.plant, sample.speed, sample.acceleration, 0.60
        )
        fw_weight = 1.0 - weight
        u_ref[stage, 1] = (
            (1.0 - fw_weight) * mc
            + transition_pusher_trim(sample.speed, weight)
        )
        if gate.state == "front_transition":
            u_ref[stage, 1] = gate.front_pusher_command
    parameters = np.zeros((controller.N + 1, controller.model.parameter_size))
    parameters[:, 3] = elevators
    parameters[:, 4] = weight
    return np.vstack(x_ref), u_ref, parameters


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    controller = StandardVtolNmpc(
        build_directory=root / "build/standard_vtol_nmpc_gate_d_pusher_030",
        control_lower_bounds=np.array([0.0, 0.0, -0.5, -0.5, -0.3]),
        control_upper_bounds=np.array([0.65, 0.60, 0.5, 0.5, 0.3]),
    )
    plant = StandardVtolTransitionRateModel(controller.model.plant)
    state = plant.hover_state()
    state[2] = 30.0
    hold = state.copy()
    command = plant.hover_control()
    gate = GateDStateMachine()
    gate.start(0.0)
    vtol_state = VTOL_MC
    transition_started = math.nan
    solver_failures = 0
    maxima = dict(
        speed=0.0,
        altitude=0.0,
        altitude_signed=0.0,
        altitude_time=0.0,
        altitude_phase="none",
        tilt=0.0,
        pusher=0.0,
    )

    for index in range(round(90.0 / controller.dt)):
        elapsed = index * controller.dt
        airspeed = max(0.0, float(state[3]))

        if vtol_state == VTOL_TRANSITION_TO_FW:
            if airspeed >= 10.0 and elapsed - transition_started >= 2.0:
                vtol_state = VTOL_FW
        elif vtol_state == VTOL_TRANSITION_TO_MC:
            if elapsed - transition_started >= 3.0:
                vtol_state = VTOL_MC

        update = gate.update(
            elapsed, vtol_state, state[3], airspeed, command[1]
        )
        if update.transition_request == VTOL_FW:
            vtol_state = VTOL_TRANSITION_TO_FW
            transition_started = elapsed
        elif update.transition_request == VTOL_MC:
            vtol_state = VTOL_TRANSITION_TO_MC
            transition_started = elapsed
        if update.failed:
            raise SystemExit(f"transition_gate_d_offline=FAIL:{gate.abort_reason}")
        if update.completed:
            break

        x_ref, u_ref, parameters = references(
            controller, gate, hold, state, elapsed, vtol_state
        )
        solution = controller.solve(state, x_ref, u_ref, parameters)
        if solution.status != 0 or not np.all(np.isfinite(solution.control)):
            solver_failures += 1
            if solver_failures >= 3:
                raise SystemExit("transition_gate_d_offline=FAIL:solver")
            continue

        requested = solution.control.copy()
        if gate.state == "mc_accelerate":
            requested[0], _ = pretransition_lift_command(
                plant.plant,
                state[2] - hold[2],
                state[5],
                state[3],
                maximum_unloading=0.020,
            )
        else:
            requested[0] = vertical_hover_lift(
                plant.plant, state[2] - hold[2], state[5]
            )
        if gate.state == "front_transition":
            requested[1] = max(requested[1], gate.front_pusher_command)
        previous = command.copy()
        pusher_limit = (
            gate.mc_pusher_limit
            if gate.state == "mc_accelerate"
            else 0.60
        )
        pusher_slew = 0.05 if gate.state == "mc_accelerate" else 0.33
        command = limit_transition_command(
            previous,
            requested,
            controller.dt,
            pusher_limit=pusher_limit,
            pusher_slew=pusher_slew,
            apply_lift_blend=False,
        )
        command = govern_pusher_forward_lateral(
            previous, command, state[1] - hold[1], state[4], 0.0, controller.dt
        )
        qw, qx, qy, qz = state[6:10]
        pitch = math.asin(
            np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0)
        )
        rw, rx, ry, rz = x_ref[0, 6:10]
        reference_pitch = math.asin(
            np.clip(2.0 * (rw * ry - rz * rx), -1.0, 1.0)
        )
        command = govern_transition_pitch(
            previous, command, pitch, reference_pitch, controller.dt
        )
        command = govern_transition_speed(
            previous,
            command,
            state[3],
            x_ref[0, 3],
            controller.dt,
        )
        if elapsed < 0.5:
            command = plant.hover_control()
        state = rk4_step(
            plant, state, command, parameters[0], controller.dt
        )
        qw, qx, qy, qz = state[6:10]
        roll = math.atan2(
            2.0 * (qw * qx + qy * qz),
            1.0 - 2.0 * (qx * qx + qy * qy),
        )
        pitch = math.asin(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0))
        maxima["speed"] = max(maxima["speed"], float(state[3]))
        altitude_error = float(state[2] - hold[2])
        if abs(altitude_error) > maxima["altitude"]:
            maxima["altitude"] = abs(altitude_error)
            maxima["altitude_signed"] = altitude_error
            maxima["altitude_time"] = elapsed
            maxima["altitude_phase"] = gate.state
        maxima["tilt"] = max(maxima["tilt"], math.degrees(max(abs(roll), abs(pitch))))
        maxima["pusher"] = max(maxima["pusher"], float(command[1]))
    else:
        raise SystemExit("transition_gate_d_offline=FAIL:did_not_complete")

    passed = (
        gate.state == "complete"
        and vtol_state == VTOL_MC
        and solver_failures == 0
        and maxima["speed"] <= 14.0
        and maxima["altitude"] <= 2.0
        and maxima["tilt"] <= 20.0
        and abs(command[1]) <= 0.005
    )
    print(f"gate_d_state={gate.state}")
    print(f"final_vtol_state={vtol_state}")
    print(f"duration={elapsed:.2f}s")
    print(f"solver_failures={solver_failures}")
    print(f"maximum_speed={maxima['speed']:.3f}m/s")
    print(f"maximum_altitude_error={maxima['altitude']:.3f}m")
    print(
        "maximum_altitude_location="
        f"{maxima['altitude_signed']:.3f}m@{maxima['altitude_time']:.2f}s"
        f",phase={maxima['altitude_phase']}"
    )
    print(f"maximum_tilt={maxima['tilt']:.2f}deg")
    print(f"maximum_pusher={maxima['pusher']:.3f}")
    print(f"final_speed={state[3]:.3f}m/s")
    print(f"final_pusher={command[1]:.4f}")
    if not passed:
        raise SystemExit("transition_gate_d_offline=FAIL:acceptance_envelope")
    print("transition_gate_d_offline=PASS")


if __name__ == "__main__":
    main()
