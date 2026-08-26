"""CasADi form of the reduced 10-state Standard VTOL transition model."""

from __future__ import annotations

import casadi as cs
import numpy as np

from px4_mpc.models.standard_vtol_gz_model import StandardVtolGazeboModel


def _vector(values) -> cs.DM:
    return cs.DM(np.asarray(values, dtype=float))


def _safe_unit(vector):
    return vector / cs.sqrt(cs.dot(vector, vector) + 1.0e-24)


def _rotation_matrix(quaternion):
    quaternion = quaternion / cs.sqrt(cs.dot(quaternion, quaternion) + 1.0e-24)
    w, x, y, z = (quaternion[index] for index in range(4))
    return cs.vertcat(
        cs.horzcat(1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        cs.horzcat(2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        cs.horzcat(2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def _quaternion_derivative(quaternion, body_rates):
    w, x, y, z = (quaternion[index] for index in range(4))
    p, q, r = (body_rates[index] for index in range(3))
    return 0.5 * cs.vertcat(
        -x * p - y * q - z * r,
        w * p + y * r - z * q,
        w * q + z * p - x * r,
        w * r + x * q - y * p,
    )


class StandardVtolTransitionCasadiModel:
    """Symbolic counterpart of ``StandardVtolTransitionRateModel``."""

    state_size = 10
    control_size = 5
    # [wind_W(3), scheduled elevator trim, PX4 MC lift weight]. Elevator is not directly
    # commanded by this NMPC; PX4's rate loop and allocator remain in charge.
    parameter_size = 5

    def __init__(self, plant: StandardVtolGazeboModel | None = None) -> None:
        self.name = "standard_vtol_transition_rate_model"
        self.plant = StandardVtolGazeboModel() if plant is None else plant

    @staticmethod
    def _coefficient(alpha, slope, stall_slope, alpha_stall):
        above = slope * alpha_stall + stall_slope * (alpha - alpha_stall)
        below = -slope * alpha_stall + stall_slope * (alpha + alpha_stall)
        return cs.if_else(
            alpha > alpha_stall,
            above,
            cs.if_else(alpha < -alpha_stall, below, slope * alpha),
        )

    def _motor_force(
        self, controls, velocity_body, body_rates, wind_body, lift_weight
    ):
        collective = cs.fmin(1.0, cs.fmax(0.0, lift_weight)) * controls[0]
        pusher = controls[1]
        commands = [collective] * 4 + [pusher]
        total = cs.MX.zeros(3, 1)
        for command, motor in zip(commands, self.plant.motors):
            command = cs.fmin(1.0, cs.fmax(0.0, command))
            speed = motor.minimum_speed + command * (
                motor.maximum_speed - motor.minimum_speed
            )
            force_axis = _vector(motor.force_axis_b)
            force_axis /= np.linalg.norm(motor.force_axis_b)
            drag_axis = _vector(motor.drag_axis_b)
            drag_axis /= np.linalg.norm(motor.drag_axis_b)
            position = _vector(motor.position_b) - _vector(self.plant.center_of_mass_b)
            velocity_at_rotor = velocity_body + cs.cross(body_rates, position) - wind_body
            perpendicular = velocity_at_rotor - cs.dot(velocity_at_rotor, drag_axis) * drag_axis
            thrust = motor.motor_constant * speed**2 * force_axis
            drag = -cs.fabs(speed) * motor.rotor_drag_coefficient * perpendicular
            total += thrust + drag
        return total

    def _surface_force(
        self, surface, deflection, velocity_body, body_rates, wind_body
    ):
        cp = _vector(surface.cp_b) - _vector(self.plant.center_of_mass_b)
        velocity = velocity_body + cs.cross(body_rates, cp) - wind_body
        speed = cs.sqrt(cs.dot(velocity, velocity) + 1.0e-24)
        forward = _vector(surface.forward_b)
        upward = _vector(surface.upward_b)
        spanwise = _safe_unit(cs.cross(forward, upward))
        velocity_unit = velocity / speed
        sin_sweep = cs.fmin(1.0, cs.fmax(-1.0, cs.dot(spanwise, velocity_unit)))
        cos2_sweep = 1.0 - sin_sweep**2
        velocity_plane = velocity - cs.dot(velocity, spanwise) * spanwise
        speed_plane = cs.sqrt(cs.dot(velocity_plane, velocity_plane) + 1.0e-24)
        drag_direction = -velocity_plane / speed_plane
        lift_direction = _safe_unit(cs.cross(spanwise, velocity_plane))
        # This signed angle matches the Gazebo acos + sign construction while
        # avoiding acos' singular derivative at level flight (alpha=0).
        alpha = surface.alpha_zero + cs.atan2(
            cs.dot(lift_direction, forward),
            cs.dot(lift_direction, upward),
        )
        alpha = cs.if_else(
            alpha > 0.5 * np.pi,
            alpha - np.pi,
            cs.if_else(alpha < -0.5 * np.pi, alpha + np.pi, alpha),
        )
        pressure = 0.5 * surface.air_density * speed_plane**2
        cl = self._coefficient(
            alpha, surface.cla, surface.cla_stall, surface.alpha_stall
        ) * cos2_sweep
        cl = cs.if_else(alpha > surface.alpha_stall, cs.fmax(0.0, cl), cl)
        cl = cs.if_else(alpha < -surface.alpha_stall, cs.fmin(0.0, cl), cl)
        cl += surface.control_rad_to_cl * cs.fmin(
            self.plant.surface_deflection_limit,
            cs.fmax(-self.plant.surface_deflection_limit, deflection),
        )
        cd = cs.fabs(
            self._coefficient(
                alpha, surface.cda, surface.cda_stall, surface.alpha_stall
            )
            * cos2_sweep
        )
        force = pressure * surface.area * (
            cl * lift_direction + cd * drag_direction
        )
        active = cs.logic_and(speed > 0.01, cs.dot(forward, velocity) > 0.0)
        return cs.if_else(active, force, cs.MX.zeros(3, 1))

    def symbolic_dynamics(self):
        """Return symbols and explicit continuous dynamics ``(x,u,wind,f)``."""
        state = cs.MX.sym("x", self.state_size)
        control = cs.MX.sym("u", self.control_size)
        parameters = cs.MX.sym("parameters", self.parameter_size)
        wind_world = parameters[0:3]
        elevator_trim = parameters[3]
        lift_weight = parameters[4]
        velocity_world = state[3:6]
        quaternion = state[6:10]
        body_rates = control[2:5]
        rotation = _rotation_matrix(quaternion)
        velocity_body = rotation.T @ velocity_world
        wind_body = rotation.T @ wind_world

        force_body = self._motor_force(
            control, velocity_body, body_rates, wind_body, lift_weight
        )
        for index, surface in enumerate(self.plant.aero_surfaces):
            deflection = elevator_trim if index == 2 else 0.0
            force_body += self._surface_force(
                surface, deflection, velocity_body, body_rates, wind_body
            )
        acceleration = (
            rotation @ force_body / self.plant.mass
            + cs.vertcat(0.0, 0.0, -self.plant.gravity)
        )
        dynamics = cs.vertcat(
            velocity_world,
            acceleration,
            _quaternion_derivative(quaternion, body_rates),
        )
        return state, control, parameters, dynamics

    def function(self) -> cs.Function:
        """Create a numerical CasADi function for tests and simulation."""
        state, control, wind, dynamics = self.symbolic_dynamics()
        return cs.Function(self.name, [state, control, wind], [dynamics])

    def get_acados_model(self):
        """Build an AcadosModel, importing acados_template only when requested."""
        try:
            from acados_template import AcadosModel
        except ImportError as error:
            raise RuntimeError(
                "acados_template is unavailable. Source/install the local acados "
                "Python interface before building the OCP."
            ) from error
        state, control, wind, dynamics = self.symbolic_dynamics()
        state_dot = cs.MX.sym("xdot", self.state_size)
        model = AcadosModel()
        model.name = self.name
        model.x = state
        model.xdot = state_dot
        model.u = control
        model.p = wind
        model.f_expl_expr = dynamics
        model.f_impl_expr = state_dot - dynamics
        return model
