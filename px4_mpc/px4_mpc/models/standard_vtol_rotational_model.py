"""Torque-informed Standard VTOL transition validation model."""

from __future__ import annotations

import numpy as np

from px4_mpc.models.frames import flu_to_frd
from px4_mpc.models.standard_vtol_gz_model import (
    StandardVtolGazeboModel,
    quaternion_derivative,
    quaternion_to_rotation,
)
from px4_mpc.models.standard_vtol_rate_control import StandardVtolRateControlModel


class StandardVtolTorqueInformedModel:
    """16-state rigid-body model with PX4 rate/allocation and surface dynamics.

    State is ``[position_W(3), velocity_W(3), quaternion_WB(4), omega_B(3),
    surface_angle(3)]`` in Gazebo ENU/FLU. Control is ``[collective, pusher,
    p_sp, q_sp, r_sp, lambda]``. Individual rotor speeds are deliberately not
    OCP states; the three surface states are retained because the Gazebo joint
    response is about two orders of magnitude slower than the motor response.
    """

    state_size = 16
    control_size = 6

    # Identified on run_01 with identical left/right parameters enforced, then
    # frozen and checked on run_02. The approximately 1 s elevon response also
    # agrees with the SDF joint damping and default position-controller scale.
    surface_time_constants = np.ones(3)
    surface_static_gains = np.ones(3)

    # Effective dimensional pitch-moment coefficients identified on run_01
    # and frozen before run_02 validation. Features are qbar times
    # [qbar, qbar*alpha, qbar*alpha*abs(alpha), qbar*q/V,
    #  qbar*elevator_angle, motor_pitch_moment].  The final term captures the
    # coupled wing-lift cancellation of the rotor-drag pitch moment.  Treating
    # those two large opposing moments independently produced a poor blend
    # rollout even when future logged actuator commands were supplied.
    pitch_moment_coefficients_blend = np.array([
        0.000339290375,
        -0.0100809491,
        0.0777383470,
        0.0,
        -0.000615907819,
        -1.01509702,
    ])
    pitch_moment_coefficients_fw = np.array([
        0.000979764100,
        0.0144568765,
        -0.370752710,
        -0.00604469791,
        -0.00396339069,
        0.0,
    ])

    # Least-squares reconstruction of PX4 control allocation on training ULog
    # run_01. Rows are [collective, tau_x, tau_y, tau_z, bias], columns are
    # lift motors 0..3. Untouched run_02 RMSE is below 0.002 command units.
    mc_allocation = np.array([
        [0.99796018, 0.99357645, 0.99554967, 0.99619316],
        [-0.37841114, 0.21905099, 0.29112765, -0.45550388],
        [0.61663248, -0.52927875, 0.77766407, -0.69447365],
        [0.80931694, 0.83840345, -0.85211059, -0.98240416],
        [0.00100926, 0.00342638, 0.00235148, 0.00197531],
    ])

    # PX4 Standard VTOL CA_SV mapping: left/right elevons provide opposing
    # roll and the elevator provides pitch. Maximum simulated angle is 45 deg.
    fw_surface_allocation = np.array([
        [-np.pi / 4.0, np.pi / 4.0, 0.0],
        [0.0, 0.0, np.pi / 4.0],
        [0.0, 0.0, 0.0],
    ])

    def __init__(
        self,
        plant: StandardVtolGazeboModel | None = None,
        rate_control: StandardVtolRateControlModel | None = None,
    ) -> None:
        self.plant = plant or StandardVtolGazeboModel()
        self.rate_control = rate_control or StandardVtolRateControlModel()

    def actuator_commands(
        self,
        state: np.ndarray,
        control: np.ndarray,
        angular_acceleration_flu: np.ndarray | None = None,
        mc_integrator_frd: np.ndarray | None = None,
        fw_integrator_frd: np.ndarray | None = None,
        fw_compression: np.ndarray | None = None,
        calibrated_airspeed: float | None = None,
        fw_allocation_weight: float | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Return motor commands, surface angles, and virtual MC/FW torques."""
        state = np.asarray(state, dtype=float)
        control = np.asarray(control, dtype=float)
        if state.shape != (self.state_size,):
            raise ValueError(f"state must have shape ({self.state_size},)")
        if control.shape != (self.control_size,):
            raise ValueError(f"control must have shape ({self.control_size},)")

        omega_flu = state[10:13]
        omega_sp_flu = control[2:5]
        omega_frd = flu_to_frd(omega_flu)
        omega_sp_frd = flu_to_frd(omega_sp_flu)
        acceleration_frd = flu_to_frd(
            np.zeros(3) if angular_acceleration_flu is None else angular_acceleration_flu
        )
        mc_integrator = np.zeros(3) if mc_integrator_frd is None else np.asarray(mc_integrator_frd, dtype=float)
        fw_integrator = np.zeros(3) if fw_integrator_frd is None else np.asarray(fw_integrator_frd, dtype=float)
        compression = np.ones(3) if fw_compression is None else np.asarray(fw_compression, dtype=float)

        rotation = quaternion_to_rotation(state[6:10])
        velocity_body = rotation.T @ state[3:6]
        airspeed = float(np.linalg.norm(velocity_body)) if calibrated_airspeed is None else float(calibrated_airspeed)
        mc_torque = self.rate_control.mc_torque(
            omega_frd, omega_sp_frd, acceleration_frd, mc_integrator
        )
        fw_torque = self.rate_control.fw_torque(
            omega_frd, omega_sp_frd, acceleration_frd, fw_integrator,
            airspeed, compression,
        )

        allocation_weight = float(np.clip(control[5], 0.0, 1.0))
        allocation_input = np.r_[
            allocation_weight * float(np.clip(control[0], 0.0, 1.0)),
            allocation_weight * mc_torque,
            1.0,
        ]
        lift_commands = np.clip(allocation_input @ self.mc_allocation, 0.0, 1.0)
        motor_commands = np.r_[lift_commands, float(np.clip(control[1], 0.0, 1.0))]
        fw_weight = (
            1.0 - allocation_weight
            if fw_allocation_weight is None
            else float(np.clip(fw_allocation_weight, 0.0, 1.0))
        )
        surface_angles = np.clip(
            (fw_weight * fw_torque) @ self.fw_surface_allocation,
            -self.plant.surface_deflection_limit,
            self.plant.surface_deflection_limit,
        )
        return motor_commands, surface_angles, mc_torque, fw_torque

    def derivative(
        self,
        state: np.ndarray,
        control: np.ndarray,
        wind_w: np.ndarray | None = None,
        **controller_parameters,
    ) -> np.ndarray:
        state = np.asarray(state, dtype=float)
        control = np.asarray(control, dtype=float)
        motor_commands, surface_commands, _, _ = self.actuator_commands(
            state, control, **controller_parameters
        )
        rotation = quaternion_to_rotation(state[6:10])
        velocity_body = rotation.T @ state[3:6]
        omega_body = state[10:13]
        wind_w = np.zeros(3) if wind_w is None else np.asarray(wind_w, dtype=float)
        wind_body = rotation.T @ wind_w
        rotor_speeds = np.asarray([
            motor.target_speed(command)
            for motor, command in zip(self.plant.motors, motor_commands)
        ])
        motor_force, motor_torque = self.plant.motor_wrench(
            rotor_speeds, velocity_body, omega_body, wind_body
        )
        aero_force, aero_torque = self.aerodynamic_wrench(
            state[13:16], velocity_body, omega_body, wind_body,
            lift_fraction=float(np.clip(control[5], 0.0, 1.0)),
            motor_pitch_moment=float(motor_torque[1]),
        )
        total_force = motor_force + aero_force
        total_torque = motor_torque + aero_torque

        result = np.zeros(self.state_size)
        result[0:3] = state[3:6]
        result[3:6] = (
            rotation @ total_force / self.plant.mass
            + np.array([0.0, 0.0, -self.plant.gravity])
        )
        result[6:10] = quaternion_derivative(state[6:10], omega_body)
        result[10:13] = np.linalg.solve(
            self.plant.inertia_b,
            total_torque - np.cross(omega_body, self.plant.inertia_b @ omega_body),
        )
        result[13:16] = (
            self.surface_static_gains * surface_commands - state[13:16]
        ) / self.surface_time_constants
        return result

    def aerodynamic_wrench(
        self,
        surface_angles: np.ndarray,
        velocity_body: np.ndarray,
        omega_body: np.ndarray,
        wind_body: np.ndarray | None = None,
        lift_fraction: float = 0.0,
        motor_pitch_moment: float = 0.0,
    ) -> tuple[np.ndarray, np.ndarray]:
        """SDF force/roll/yaw plus the ULog-identified pitch moment."""
        wind_body = np.zeros(3) if wind_body is None else np.asarray(wind_body, dtype=float)
        relative_velocity = np.asarray(velocity_body, dtype=float) - wind_body
        force, torque = self.plant.aerodynamic_wrench(
            surface_angles, velocity_body, omega_body, wind_body
        )
        forward_speed = max(float(relative_velocity[0]), 0.0)
        alpha = float(np.arctan2(-relative_velocity[2], max(forward_speed, 0.1)))
        dynamic_pressure = (
            0.5 * self.plant.aero_surfaces[0].air_density * forward_speed * forward_speed
        )
        features = np.r_[dynamic_pressure * np.asarray([
            1.0,
            alpha,
            alpha * abs(alpha),
            float(omega_body[1]) / max(forward_speed, 1.0),
            float(surface_angles[2]),
        ]), float(motor_pitch_moment)]
        lift_fraction = float(np.clip(lift_fraction, 0.0, 1.0))
        # Keep the stable mixed-authority fit through most of the blend and
        # move continuously to the FW fit only near complete lift unloading.
        # Direct linear interpolation across [0, 1] was rejected because its
        # 0.5 s blend pitch rollout became unstable.
        blend_weight = float(np.clip(lift_fraction / 0.2, 0.0, 1.0))
        blend_weight = blend_weight * blend_weight * (3.0 - 2.0 * blend_weight)
        coefficients = (
            blend_weight * self.pitch_moment_coefficients_blend
            + (1.0 - blend_weight) * self.pitch_moment_coefficients_fw
        )
        torque[1] = float(features @ coefficients)
        return force, torque

    def step_surface_state(
        self, surface_state: np.ndarray, surface_command: np.ndarray, dt: float
    ) -> np.ndarray:
        """Exact zero-order-hold step of the identified first-order joints."""
        surface_state = np.asarray(surface_state, dtype=float)
        surface_command = np.asarray(surface_command, dtype=float)
        if surface_state.shape != (3,) or surface_command.shape != (3,):
            raise ValueError("surface state and command must have shape (3,)")
        if dt < 0.0:
            raise ValueError("dt must be nonnegative")
        decay = np.exp(-float(dt) / self.surface_time_constants)
        return (
            decay * surface_state
            + (1.0 - decay) * self.surface_static_gains * surface_command
        )

    def hover_state(self) -> np.ndarray:
        state = np.zeros(self.state_size)
        state[6] = 1.0
        return state

    def hover_control(self) -> np.ndarray:
        return np.array([self.plant.hover_command, 0.0, 0.0, 0.0, 0.0, 1.0])
