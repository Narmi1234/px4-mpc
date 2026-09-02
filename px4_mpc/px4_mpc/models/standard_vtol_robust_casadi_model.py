"""Sixteen-state CasADi model for robust Standard VTOL transition NMPC."""

from __future__ import annotations

import casadi as cs
import numpy as np

from px4_mpc.models.standard_vtol_casadi_model import (
    StandardVtolTransitionCasadiModel,
    _quaternion_derivative,
    _rotation_matrix,
    _vector,
)
from px4_mpc.models.standard_vtol_pitch_rate_model import (
    StandardVtolPitchRateLpvModel,
)


class StandardVtolRobustCasadiModel(StandardVtolTransitionCasadiModel):
    """Robust-NMPC model with body rates, surface lag and owned allocation.

    State is ``[position_W(3), velocity_W(3), quaternion_WB(4),
    omega_B_FLU(3), surface_angles(3)]``. Control is ``[collective, pusher,
    p_sp, q_sp, r_sp, lambda]``. Parameters are ``[wind_W(3), force_bias_x,
    force_bias_z, pitch_rate_disturbance_FRD]``.
    """

    state_size = 16
    control_size = 6
    parameter_size = 6
    surface_time_constant = 1.0
    roll_rate_gain = 6.0
    yaw_rate_gain = 4.0
    elevator_feedforward_limit = 0.25
    pitch_disturbance_bound = StandardVtolPitchRateLpvModel.pitch_disturbance_bound

    # Force-balanced Gazebo trim corridor. The bounded feed-forward component
    # is sent explicitly through VtolNmpcAllocationSetpoint; q_sp supplies the
    # feedback increment around it. It remains scheduled from the predicted
    # airspeed rather than becoming a seventh optimized NMPC input.
    elevator_trim_speed_nodes = np.arange(5.0, 23.0)
    elevator_trim_nodes = np.deg2rad(
        [
            43.3170391643, 43.4303645063, 43.2066040519,
            44.2501496316, 43.8751953333, 41.6336729723,
            32.0567524881, 25.1395207718, 19.9749245714,
            16.0132129747, 12.9051872155, 10.4202518173,
            8.4010637420, 6.7372163334, 5.3493413743,
            4.1791660275, 3.1831034277, 2.3280098741,
        ]
    )

    def __init__(self, plant=None) -> None:
        super().__init__(plant)
        self.name = "standard_vtol_robust_transition_model"

    @staticmethod
    def _clip(value, lower, upper):
        return cs.fmin(upper, cs.fmax(lower, value))

    def _pitch_rate_derivative(
        self, pitch_rate_flu, pitch_rate_sp_flu, airspeed, lift_fraction,
        alpha, pitch_flu, disturbance_frd,
    ):
        model = StandardVtolPitchRateLpvModel
        speed = self._clip(airspeed / model.speed_upper_node, 0.0, 1.0)
        lift = self._clip(lift_fraction, 0.0, 1.0)
        mu = 1.0 - lift
        basis = cs.vertcat(
            (1.0 - speed) * (1.0 - mu),
            speed * (1.0 - mu),
            (1.0 - speed) * mu,
            speed * mu,
        )
        damping = cs.dot(basis, _vector(model.damping_nodes))
        input_gain = cs.dot(basis, _vector(model.input_gain_nodes))
        transition = 4.0 * mu * (1.0 - mu)
        bias = transition * cs.dot(
            cs.vertcat(1.0, alpha, -pitch_flu),
            _vector(model.transition_bias_coefficients),
        )
        bounded_disturbance = self._clip(
            disturbance_frd,
            -model.pitch_disturbance_bound,
            model.pitch_disturbance_bound,
        )
        # Identification is in PX4 FRD, while this plant is Gazebo FLU:
        # q_FRD=-q_FLU, pitch_FRD=-pitch_FLU and qdot_FLU=-qdot_FRD.
        return (
            -damping * pitch_rate_flu
            + input_gain * pitch_rate_sp_flu
            - bias
            - bounded_disturbance
        )

    @classmethod
    def _elevator_trim(cls, airspeed):
        """Piecewise-linear PX4/Gazebo FW elevator feed-forward trim."""
        speeds = cls.elevator_trim_speed_nodes
        values = cls.elevator_trim_nodes
        result = float(values[0])
        for index in range(len(speeds) - 1):
            fraction = (airspeed - speeds[index]) / (
                speeds[index + 1] - speeds[index]
            )
            segment = values[index] + fraction * (
                values[index + 1] - values[index]
            )
            result = cs.if_else(airspeed >= speeds[index], segment, result)
        return cls._clip(result, float(values[-1]), float(np.max(values)))

    @classmethod
    def elevator_feedforward(cls, airspeed: float, lift_fraction: float) -> float:
        """Normalized trim sent through the explicit PX4 elevator channel."""
        trim = float(
            np.interp(
                float(airspeed),
                cls.elevator_trim_speed_nodes,
                cls.elevator_trim_nodes,
            )
        )
        requested = (1.0 - np.clip(lift_fraction, 0.0, 1.0)) * trim / (
            np.pi / 4.0
        )
        return float(
            np.clip(
                requested,
                -cls.elevator_feedforward_limit,
                cls.elevator_feedforward_limit,
            )
        )

    def symbolic_dynamics(self):
        state = cs.MX.sym("x", self.state_size)
        control = cs.MX.sym("u", self.control_size)
        parameters = cs.MX.sym("parameters", self.parameter_size)
        velocity_world = state[3:6]
        quaternion = state[6:10]
        body_rates = state[10:13]
        surface_angles = state[13:16]
        collective = self._clip(control[0], 0.0, 1.0)
        pusher = self._clip(control[1], 0.0, 1.0)
        rate_sp = control[2:5]
        lift_fraction = self._clip(control[5], 0.0, 1.0)
        wind_world = parameters[0:3]

        rotation = _rotation_matrix(quaternion)
        velocity_body = rotation.T @ velocity_world
        wind_body = rotation.T @ wind_world
        relative_velocity = velocity_body - wind_body
        forward_speed = cs.fmax(relative_velocity[0], 0.0)
        airspeed = cs.sqrt(cs.dot(relative_velocity, relative_velocity) + 1.0e-12)
        alpha = cs.atan2(-relative_velocity[2], cs.fmax(forward_speed, 0.1))
        w, x, y, z = (quaternion[index] for index in range(4))
        pitch_flu = cs.asin(self._clip(2.0 * (w * y - z * x), -1.0, 1.0))

        motor_control = cs.vertcat(
            collective, pusher, rate_sp[0], rate_sp[1], rate_sp[2]
        )
        force_body = self._motor_force(
            motor_control, velocity_body, body_rates, wind_body, lift_fraction
        )
        for index, surface in enumerate(self.plant.aero_surfaces):
            force_body += self._surface_force(
                surface, surface_angles[index], velocity_body, body_rates,
                wind_body,
            )
        force_body += cs.vertcat(parameters[3], 0.0, parameters[4])
        acceleration = (
            rotation @ force_body / self.plant.mass
            + cs.vertcat(0.0, 0.0, -self.plant.gravity)
        )

        rate_derivative = cs.vertcat(
            self.roll_rate_gain * (rate_sp[0] - body_rates[0]),
            self._pitch_rate_derivative(
                body_rates[1], rate_sp[1], airspeed, lift_fraction,
                alpha, pitch_flu, parameters[5],
            ),
            self.yaw_rate_gain * (rate_sp[2] - body_rates[2]),
        )

        # PX4 FW virtual torques in FRD, mapped to Gazebo surface angles.
        roll_error_frd = rate_sp[0] - body_rates[0]
        pitch_error_frd = -(rate_sp[1] - body_rates[1])
        fw_weight = 1.0 - lift_fraction
        roll_torque = self._clip(0.30 * roll_error_frd, -1.0, 1.0)
        pitch_torque = self._clip(0.90 * pitch_error_frd, -1.0, 1.0)
        elevator_trim = self._elevator_trim(airspeed)
        elevator_feedforward = self._clip(
            fw_weight * elevator_trim / (np.pi / 4.0),
            -self.elevator_feedforward_limit,
            self.elevator_feedforward_limit,
        )
        surface_command = fw_weight * cs.vertcat(
            -np.pi / 4.0 * roll_torque,
            np.pi / 4.0 * roll_torque,
            np.pi / 4.0 * pitch_torque,
        )
        surface_command[2] += np.pi / 4.0 * elevator_feedforward
        surface_derivative = (
            surface_command - surface_angles
        ) / self.surface_time_constant

        dynamics = cs.vertcat(
            velocity_world,
            acceleration,
            _quaternion_derivative(quaternion, body_rates),
            rate_derivative,
            surface_derivative,
        )
        return state, control, parameters, dynamics

    def function(self):
        state, control, parameters, dynamics = self.symbolic_dynamics()
        return cs.Function(self.name, [state, control, parameters], [dynamics])

    def get_acados_model(self):
        try:
            from acados_template import AcadosModel
        except ImportError as error:
            raise RuntimeError(
                "acados_template is unavailable; source the local acados setup"
            ) from error
        state, control, parameters, dynamics = self.symbolic_dynamics()
        state_dot = cs.MX.sym("xdot", self.state_size)
        model = AcadosModel()
        model.name = self.name
        model.x = state
        model.xdot = state_dot
        model.u = control
        model.p = parameters
        model.f_expl_expr = dynamics
        model.f_impl_expr = state_dot - dynamics
        return model

    def hover_state(self):
        state = np.zeros(self.state_size)
        state[6] = 1.0
        return state

    def hover_control(self):
        return np.array([
            self.plant.hover_command, 0.0, 0.0, 0.0, 0.0, 1.0
        ])
