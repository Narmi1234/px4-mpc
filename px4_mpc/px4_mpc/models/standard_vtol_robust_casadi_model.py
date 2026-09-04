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
    force_bias_z, pitch_rate_disturbance_FRD, mc_roll_pitch_weight,
    mc_yaw_weight]``.
    """

    state_size = 16
    control_size = 6
    parameter_size = 8
    surface_time_constant = 1.0
    roll_rate_gain = 6.0
    yaw_rate_gain = 4.0
    # Identified from the successful L3c and failed L4a SITL ULogs. With
    # normalized differential aileron delta_a, both datasets are described by
    # p_dot = -a_p*p + b_p*V^2*delta_a. The individual fits bracket these
    # nominal values (a_p=0.34..0.67, b_p=0.041..0.055).
    roll_aero_damping = 0.50
    roll_surface_gain = 0.0475
    # ULog course-rate replay gave gains 0.94 and 0.96 against
    # g*tan(phi)/V (correlation 0.96 in both runs).
    coordinated_turn_gain = 0.95
    minimum_coordinated_speed = 4.0
    fw_roll_p_gain = 0.30
    fw_roll_ff_gain = 0.10
    fw_airspeed_trim = 15.0
    fw_airspeed_stall = 7.0
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
        mc_roll_pitch_weight = self._clip(parameters[6], 0.0, 1.0)
        mc_yaw_weight = self._clip(parameters[7], 0.0, 1.0)

        rotation = _rotation_matrix(quaternion)
        velocity_body = rotation.T @ velocity_world
        wind_body = rotation.T @ wind_world
        relative_velocity = velocity_body - wind_body
        forward_speed = cs.fmax(relative_velocity[0], 0.0)
        airspeed = cs.sqrt(cs.dot(relative_velocity, relative_velocity) + 1.0e-12)
        alpha = cs.atan2(-relative_velocity[2], cs.fmax(forward_speed, 0.1))
        w, x, y, z = (quaternion[index] for index in range(4))
        pitch_flu = cs.asin(self._clip(2.0 * (w * y - z * x), -1.0, 1.0))
        roll_flu = cs.atan2(
            2.0 * (w * x + y * z),
            1.0 - 2.0 * (x * x + y * y),
        )

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

        differential_aileron = (
            surface_angles[1] - surface_angles[0]
        ) / (2.0 * (np.pi / 4.0))
        aerodynamic_roll_fraction = 1.0 - mc_roll_pitch_weight
        roll_aero_derivative = (
            -aerodynamic_roll_fraction
            * self.roll_aero_damping * body_rates[0]
            + self.roll_surface_gain * airspeed * airspeed
            * differential_aileron
        )
        # In ENU/FLU a positive right bank produces a negative yaw/course
        # rate. Keep the MC yaw actuator available, but progressively command
        # the measured coordinated-turn law as roll/pitch authority transfers.
        coordinated_yaw_rate = (
            -self.coordinated_turn_gain * self.plant.gravity
            * cs.tan(roll_flu)
            / cs.fmax(airspeed, self.minimum_coordinated_speed)
        )
        coordination_fraction = aerodynamic_roll_fraction
        yaw_rate_target = (
            (1.0 - coordination_fraction) * rate_sp[2]
            + coordination_fraction * coordinated_yaw_rate
        )
        rate_derivative = cs.vertcat(
            mc_roll_pitch_weight * self.roll_rate_gain
            * (rate_sp[0] - body_rates[0])
            + roll_aero_derivative,
            self._pitch_rate_derivative(
                body_rates[1], rate_sp[1], airspeed, lift_fraction,
                alpha, pitch_flu, parameters[5],
            ),
            mc_yaw_weight * self.yaw_rate_gain
            * (yaw_rate_target - body_rates[2])
            + (1.0 - mc_yaw_weight) * 2.0
            * (coordinated_yaw_rate - body_rates[2]),
        )

        # PX4 FW virtual torques in FRD, mapped to Gazebo surface angles.
        roll_error_frd = rate_sp[0] - body_rates[0]
        pitch_error_frd = -(rate_sp[1] - body_rates[1])
        # Surface authority follows the independently applied roll/pitch
        # allocation, not the vertical lift-motor fraction.
        fw_weight = 1.0 - mc_roll_pitch_weight
        # Match FixedwingRateControl's configured P+FF airspeed scaling. The
        # integral is deliberately not predicted as an unconstrained hidden
        # state; PX4 bounds it and resets it outside a fresh external transfer.
        airspeed_scaling = self.fw_airspeed_trim / cs.fmax(
            airspeed, self.fw_airspeed_stall
        )
        roll_torque = self._clip(
            (
                self.fw_roll_p_gain * roll_error_frd
                + self.fw_roll_ff_gain / airspeed_scaling * rate_sp[0]
            ) * airspeed_scaling * airspeed_scaling,
            -1.0,
            1.0,
        )
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

    def nominal_parameters(self):
        """Zero disturbances with full MC roll/pitch and yaw authority."""
        parameters = np.zeros(self.parameter_size)
        parameters[6:8] = 1.0
        return parameters
