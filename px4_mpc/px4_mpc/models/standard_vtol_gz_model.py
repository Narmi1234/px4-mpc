"""Flight-dynamics model of PX4's Gazebo ``standard_vtol``.

The parameter values in this module are a checked-in snapshot of
``PX4-Autopilot/Tools/simulation/gz/models/standard_vtol/model.sdf`` at PX4
revision ``5f1eae330b``.  The equations mirror the Gazebo Sim 8
``MulticopterMotorModel`` and ``LiftDrag`` systems closely enough for
controller design and model-vs-SITL validation.  Contacts, sensors, rotor
gyroscopic inertia and the joint physics used only to render spinning
propellers are intentionally outside this flight-dynamics model.

Frames
------
``W`` is the Gazebo world frame (ENU), and ``B`` is the Gazebo body frame
(FLU: x forward, y left, z up).  The quaternion is scalar-first and rotates
vectors from B to W.

Full plant state (18)
---------------------
``[p_W(3), v_W(3), q_WB(4), omega_B(3), rotor_speed(5)]``

Full plant control (8)
----------------------
``[lift_motor_0..3, pusher_motor, left_elevon, right_elevon, elevator]``.
Motor controls are normalized to [0, 1]; surface controls are joint angles
in radians.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


PX4_SOURCE_REVISION = "5f1eae330b"
PX4_SOURCE_RELATIVE_PATH = "Tools/simulation/gz/models/standard_vtol/model.sdf"

STATE_SIZE = 18
CONTROL_SIZE = 8


def _vec3(values: Iterable[float]) -> np.ndarray:
    return np.asarray(tuple(values), dtype=float)


def _unit(vector: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(vector)
    if norm <= 1.0e-12:
        return np.zeros(3)
    return vector / norm


def _rotation_y(angle: float) -> np.ndarray:
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return np.array(
        [[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]]
    )


def quaternion_to_rotation(q_wb: np.ndarray) -> np.ndarray:
    """Return the B-to-W rotation matrix for a scalar-first quaternion."""
    q_wb = np.asarray(q_wb, dtype=float)
    norm = np.linalg.norm(q_wb)
    if norm <= 1.0e-12:
        raise ValueError("quaternion norm must be nonzero")
    w, x, y, z = q_wb / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def quaternion_derivative(q_wb: np.ndarray, omega_b: np.ndarray) -> np.ndarray:
    """Quaternion derivative for body angular velocity expressed in B."""
    w, x, y, z = np.asarray(q_wb, dtype=float)
    p, q, r = np.asarray(omega_b, dtype=float)
    return 0.5 * np.array(
        [
            -x * p - y * q - z * r,
            w * p + y * r - z * q,
            w * q + z * p - x * r,
            w * r + x * q - y * p,
        ]
    )


@dataclass(frozen=True)
class MotorParameters:
    """Parameters of one Gazebo MulticopterMotorModel plugin."""

    name: str
    position_b: tuple[float, float, float]
    force_axis_b: tuple[float, float, float]
    drag_axis_b: tuple[float, float, float]
    turning_direction: int  # Gazebo: ccw=+1, cw=-1
    minimum_speed: float
    maximum_speed: float
    motor_constant: float
    moment_constant: float
    time_constant_up: float = 0.0125
    time_constant_down: float = 0.025
    rotor_drag_coefficient: float = 0.000106428
    rolling_moment_coefficient: float = 1.0e-6
    mass: float = 0.005

    def target_speed(self, normalized_command: float) -> float:
        command = float(np.clip(normalized_command, 0.0, 1.0))
        return self.minimum_speed + command * (self.maximum_speed - self.minimum_speed)

    def normalized_command(self, speed: float) -> float:
        span = self.maximum_speed - self.minimum_speed
        return float(np.clip((speed - self.minimum_speed) / span, 0.0, 1.0))


@dataclass(frozen=True)
class AeroSurfaceParameters:
    """Parameters of one Gazebo LiftDrag plugin."""

    name: str
    cp_b: tuple[float, float, float]
    area: float
    alpha_zero: float
    control_rad_to_cl: float
    cla: float = 4.752798721
    cda: float = 0.6417112299
    cma: float = 0.0
    alpha_stall: float = 0.3391428111
    cla_stall: float = -3.85
    cda_stall: float = -0.9233984055
    cma_stall: float = 0.0
    air_density: float = 1.2041
    forward_b: tuple[float, float, float] = (1.0, 0.0, 0.0)
    upward_b: tuple[float, float, float] = (0.0, 0.0, 1.0)


@dataclass(frozen=True)
class LevelFlightTrim:
    """Zero-pitch, neutral-surface trim used to seed transition optimization."""

    airspeed: float
    collective_lift_command: float
    pusher_command: float
    aerodynamic_force_b: tuple[float, float, float]
    total_force_b: tuple[float, float, float]


_PULLER_ROTATION = _rotation_y(1.57)

LIFT_MOTORS = (
    MotorParameters("rotor_0", (0.35, -0.35, 0.07), (0.0, 0.0, 1.0), (0.0, 0.0, 1.0), +1, 10.0, 1500.0, 2.0e-5, 0.06),
    MotorParameters("rotor_1", (-0.35, 0.35, 0.07), (0.0, 0.0, 1.0), (0.0, 0.0, 1.0), +1, 10.0, 1500.0, 2.0e-5, 0.06),
    MotorParameters("rotor_2", (0.35, 0.35, 0.07), (0.0, 0.0, 1.0), (0.0, 0.0, 1.0), -1, 10.0, 1500.0, 2.0e-5, 0.06),
    MotorParameters("rotor_3", (-0.35, -0.35, 0.07), (0.0, 0.0, 1.0), (0.0, 0.0, 1.0), -1, 10.0, 1500.0, 2.0e-5, 0.06),
)

PUSHER_MOTOR = MotorParameters(
    "rotor_puller",
    (-0.22, 0.0, 0.0),
    tuple(_PULLER_ROTATION @ np.array([0.0, 0.0, 1.0])),
    (1.0, 0.0, 0.0),
    -1,
    0.0,
    3500.0,
    8.54858e-6,
    0.01,
)

MOTORS = LIFT_MOTORS + (PUSHER_MOTOR,)

AERO_SURFACES = (
    AeroSurfaceParameters("left_wing", (-0.05, 0.3, 0.05), 0.50, 0.05984281113, -1.0),
    AeroSurfaceParameters("right_wing", (-0.05, -0.3, 0.05), 0.50, 0.05984281113, -1.0),
    AeroSurfaceParameters("elevator", (-0.5, 0.0, 0.0), 0.01, -0.2, -12.0),
)


class StandardVtolGazeboModel:
    """Numerical 6-DoF plant corresponding to the PX4 Gazebo model."""

    base_mass = 5.0
    base_inertia_b = np.diag([0.477708333333, 0.341666666667, 0.811041666667])
    gravity = 9.81
    surface_mass = 1.0e-8
    surface_deflection_limit = 0.78

    def __init__(self) -> None:
        self.motors = MOTORS
        self.aero_surfaces = AERO_SURFACES
        self.mass = self.base_mass + sum(motor.mass for motor in self.motors) + 3.0 * self.surface_mass
        self.center_of_mass_b = self._composite_center_of_mass()
        self.inertia_b = self._composite_inertia()
        self.inertia_b_inverse = np.linalg.inv(self.inertia_b)

    def _composite_center_of_mass(self) -> np.ndarray:
        first_moment = np.zeros(3)
        for motor in self.motors:
            first_moment += motor.mass * _vec3(motor.position_b)
        return first_moment / self.mass

    def _composite_inertia(self) -> np.ndarray:
        center = self.center_of_mass_b
        inertia = self.base_inertia_b + self.base_mass * (
            np.dot(center, center) * np.eye(3) - np.outer(center, center)
        )
        rotor_inertia = np.diag([9.75e-7, 0.000166704, 0.000167604])
        for index, motor in enumerate(self.motors):
            rotation = _PULLER_ROTATION if index == 4 else np.eye(3)
            local_inertia = rotation @ rotor_inertia @ rotation.T
            offset = _vec3(motor.position_b) - center
            inertia += local_inertia + motor.mass * (
                np.dot(offset, offset) * np.eye(3) - np.outer(offset, offset)
            )
        # The three control-surface inertias are 1e-6 on each diagonal in SDF.
        inertia += 3.0e-6 * np.eye(3)
        return inertia

    @property
    def hover_rotor_speed(self) -> float:
        """Collective hover speed, ignoring the sub-millimetre composite-CoM offset."""
        motor = self.motors[0]
        return math.sqrt(self.mass * self.gravity / (4.0 * motor.motor_constant))

    @property
    def hover_lift_rotor_speeds(self) -> np.ndarray:
        """Lift-rotor speeds that balance weight and all three body moments."""
        allocation = np.zeros((4, 4))
        for index, motor in enumerate(self.motors[:4]):
            axis = _unit(_vec3(motor.force_axis_b))
            position = _vec3(motor.position_b) - self.center_of_mass_b
            force_per_speed_squared = motor.motor_constant * axis
            torque_per_speed_squared = np.cross(position, force_per_speed_squared)
            torque_per_speed_squared += (
                -motor.turning_direction
                * motor.motor_constant
                * motor.moment_constant
                * axis
            )
            allocation[0, index] = force_per_speed_squared[2]
            allocation[1:4, index] = torque_per_speed_squared
        squared_speeds = np.linalg.solve(
            allocation,
            np.array([self.mass * self.gravity, 0.0, 0.0, 0.0]),
        )
        if np.any(squared_speeds <= 0.0):
            raise RuntimeError("Gazebo rotor geometry has no positive hover allocation")
        return np.sqrt(squared_speeds)

    @property
    def hover_command(self) -> float:
        return self.motors[0].normalized_command(self.hover_rotor_speed)

    def hover_state(self, position_w: Iterable[float] = (0.0, 0.0, 0.0)) -> np.ndarray:
        state = np.zeros(STATE_SIZE)
        state[0:3] = _vec3(position_w)
        state[6] = 1.0
        state[13:17] = self.hover_lift_rotor_speeds
        return state

    def hover_control(self) -> np.ndarray:
        control = np.zeros(CONTROL_SIZE)
        control[0:4] = [
            motor.normalized_command(speed)
            for motor, speed in zip(self.motors[:4], self.hover_lift_rotor_speeds)
        ]
        return control

    def motor_speed_derivative(self, speeds: np.ndarray, motor_commands: np.ndarray) -> np.ndarray:
        result = np.zeros(5)
        for index, (speed, command, motor) in enumerate(zip(speeds, motor_commands, self.motors)):
            target = motor.target_speed(command)
            tau = motor.time_constant_up if target > speed else motor.time_constant_down
            result[index] = (target - speed) / tau
        return result

    def motor_wrench(
        self,
        speeds: np.ndarray,
        velocity_b: np.ndarray,
        omega_b: np.ndarray,
        wind_b: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return total motor force and torque about the composite CoM in B."""
        wind_b = np.zeros(3) if wind_b is None else np.asarray(wind_b, dtype=float)
        force_b = np.zeros(3)
        torque_b = np.zeros(3)
        for speed, motor in zip(speeds, self.motors):
            position = _vec3(motor.position_b) - self.center_of_mass_b
            force_axis = _unit(_vec3(motor.force_axis_b))
            drag_axis = _unit(_vec3(motor.drag_axis_b))
            thrust = motor.motor_constant * float(speed) ** 2
            rotor_force = thrust * force_axis

            velocity_at_rotor = velocity_b + np.cross(omega_b, position) - wind_b
            velocity_perpendicular = velocity_at_rotor - np.dot(velocity_at_rotor, drag_axis) * drag_axis
            rotor_drag = -abs(float(speed)) * motor.rotor_drag_coefficient * velocity_perpendicular
            rolling_moment = -abs(float(speed)) * motor.rolling_moment_coefficient * velocity_perpendicular

            combined_force = rotor_force + rotor_drag
            reaction_torque = -motor.turning_direction * thrust * motor.moment_constant * force_axis
            force_b += combined_force
            torque_b += np.cross(position, combined_force) + reaction_torque + rolling_moment
        return force_b, torque_b

    @staticmethod
    def _coefficient(alpha: float, slope: float, stall_slope: float, alpha_stall: float) -> float:
        if alpha > alpha_stall:
            return slope * alpha_stall + stall_slope * (alpha - alpha_stall)
        if alpha < -alpha_stall:
            return -slope * alpha_stall + stall_slope * (alpha + alpha_stall)
        return slope * alpha

    def surface_wrench(
        self,
        surface: AeroSurfaceParameters,
        deflection: float,
        velocity_b: np.ndarray,
        omega_b: np.ndarray,
        wind_b: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Mirror Gazebo Sim 8 LiftDrag for one surface, expressed in B."""
        wind_b = np.zeros(3) if wind_b is None else np.asarray(wind_b, dtype=float)
        cp = _vec3(surface.cp_b) - self.center_of_mass_b
        velocity = velocity_b + np.cross(omega_b, cp) - wind_b
        if np.linalg.norm(velocity) <= 0.01:
            return np.zeros(3), np.zeros(3)

        forward = _unit(_vec3(surface.forward_b))
        upward = _unit(_vec3(surface.upward_b))
        if np.dot(forward, velocity) <= 0.0:
            return np.zeros(3), np.zeros(3)

        velocity_unit = _unit(velocity)
        spanwise = _unit(np.cross(forward, upward))
        sin_sweep = float(np.clip(np.dot(spanwise, velocity_unit), -1.0, 1.0))
        cos2_sweep = 1.0 - sin_sweep * sin_sweep
        velocity_plane = velocity - np.dot(velocity, spanwise) * spanwise
        speed_plane = np.linalg.norm(velocity_plane)
        if speed_plane <= 1.0e-12:
            return np.zeros(3), np.zeros(3)

        drag_direction = -velocity_plane / speed_plane
        lift_direction = _unit(np.cross(spanwise, velocity_plane))
        # Signed atan2 is equivalent to the Gazebo acos + sign construction,
        # but remains differentiable at alpha=0 for the CasADi counterpart.
        alpha = surface.alpha_zero + math.atan2(
            float(np.dot(lift_direction, forward)),
            float(np.dot(lift_direction, upward)),
        )
        while abs(alpha) > 0.5 * math.pi:
            alpha = alpha - math.pi if alpha > 0.0 else alpha + math.pi

        dynamic_pressure = 0.5 * surface.air_density * speed_plane * speed_plane
        cl = self._coefficient(alpha, surface.cla, surface.cla_stall, surface.alpha_stall) * cos2_sweep
        if alpha > surface.alpha_stall:
            cl = max(0.0, cl)
        elif alpha < -surface.alpha_stall:
            cl = min(0.0, cl)
        cl += surface.control_rad_to_cl * float(deflection)

        cd = abs(self._coefficient(alpha, surface.cda, surface.cda_stall, surface.alpha_stall) * cos2_sweep)
        cm = self._coefficient(alpha, surface.cma, surface.cma_stall, surface.alpha_stall) * cos2_sweep

        lift = cl * dynamic_pressure * surface.area * lift_direction
        drag = cd * dynamic_pressure * surface.area * drag_direction
        force = lift + drag
        moment = cm * dynamic_pressure * surface.area * spanwise
        return force, moment + np.cross(cp, force)

    def aerodynamic_wrench(
        self,
        deflections: np.ndarray,
        velocity_b: np.ndarray,
        omega_b: np.ndarray,
        wind_b: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        force_b = np.zeros(3)
        torque_b = np.zeros(3)
        for surface, deflection in zip(self.aero_surfaces, deflections):
            limited_deflection = float(
                np.clip(
                    deflection,
                    -self.surface_deflection_limit,
                    self.surface_deflection_limit,
                )
            )
            force, torque = self.surface_wrench(
                surface,
                limited_deflection,
                velocity_b,
                omega_b,
                wind_b,
            )
            force_b += force
            torque_b += torque
        return force_b, torque_b

    def nominal_level_flight_trim(self, airspeed: float) -> LevelFlightTrim:
        """Return a simple level-flight trim at zero pitch and neutral surfaces.

        This is an initial guess, not a complete trim solver: it balances body-x
        drag with the pusher and body-z weight with the four lift rotors.  Once
        wing lift exceeds weight, collective lift is clamped to zero and a
        pitched trim must be found by the NMPC trajectory-generation stage.
        """
        if airspeed < 0.0:
            raise ValueError("airspeed must be nonnegative")
        aero_force, _ = self.aerodynamic_wrench(
            np.zeros(3),
            np.array([airspeed, 0.0, 0.0]),
            np.zeros(3),
        )

        pusher = self.motors[4]
        pusher_axis = _unit(_vec3(pusher.force_axis_b))
        required_pusher_thrust = max(0.0, -aero_force[0] / pusher_axis[0])
        pusher_speed = math.sqrt(required_pusher_thrust / pusher.motor_constant)
        pusher_command = pusher.normalized_command(pusher_speed)
        actual_pusher_speed = pusher.target_speed(pusher_command)
        pusher_force = pusher.motor_constant * actual_pusher_speed**2 * pusher_axis

        required_lift = max(0.0, self.mass * self.gravity - aero_force[2] - pusher_force[2])
        lift_motor = self.motors[0]
        lift_speed = math.sqrt(required_lift / (4.0 * lift_motor.motor_constant))
        lift_command = lift_motor.normalized_command(lift_speed)
        actual_lift_speed = lift_motor.target_speed(lift_command)
        lift_force = np.array(
            [0.0, 0.0, 4.0 * lift_motor.motor_constant * actual_lift_speed**2]
        )
        total_force = aero_force + pusher_force + lift_force
        return LevelFlightTrim(
            airspeed=float(airspeed),
            collective_lift_command=lift_command,
            pusher_command=pusher_command,
            aerodynamic_force_b=tuple(aero_force),
            total_force_b=tuple(total_force),
        )

    def derivative(
        self,
        state: np.ndarray,
        control: np.ndarray,
        wind_w: np.ndarray | None = None,
    ) -> np.ndarray:
        """Evaluate the continuous-time 18-state flight dynamics."""
        state = np.asarray(state, dtype=float)
        control = np.asarray(control, dtype=float)
        if state.shape != (STATE_SIZE,):
            raise ValueError(f"state must have shape ({STATE_SIZE},)")
        if control.shape != (CONTROL_SIZE,):
            raise ValueError(f"control must have shape ({CONTROL_SIZE},)")

        velocity_w = state[3:6]
        q_wb = state[6:10]
        omega_b = state[10:13]
        rotor_speeds = state[13:18]
        rotation_wb = quaternion_to_rotation(q_wb)
        velocity_b = rotation_wb.T @ velocity_w
        wind_w = np.zeros(3) if wind_w is None else np.asarray(wind_w, dtype=float)
        wind_b = rotation_wb.T @ wind_w

        motor_force_b, motor_torque_b = self.motor_wrench(rotor_speeds, velocity_b, omega_b, wind_b)
        aero_force_b, aero_torque_b = self.aerodynamic_wrench(control[5:8], velocity_b, omega_b, wind_b)
        total_force_b = motor_force_b + aero_force_b
        total_torque_b = motor_torque_b + aero_torque_b

        derivative = np.zeros(STATE_SIZE)
        derivative[0:3] = velocity_w
        derivative[3:6] = rotation_wb @ total_force_b / self.mass + np.array([0.0, 0.0, -self.gravity])
        derivative[6:10] = quaternion_derivative(q_wb, omega_b)
        derivative[10:13] = self.inertia_b_inverse @ (
            total_torque_b - np.cross(omega_b, self.inertia_b @ omega_b)
        )
        derivative[13:18] = self.motor_speed_derivative(rotor_speeds, control[0:5])
        return derivative


class StandardVtolTransitionRateModel:
    """Reduced model intended for the first transition NMPC implementation.

    This is a 10-state model, not a 10-DoF model. The aircraft still has six
    physical degrees of freedom; a unit quaternion uses four state variables
    to represent its three rotational degrees of freedom.

    PX4 closes the body-rate loop. Therefore the reduced state is
    ``[p_W(3), v_W(3), q_WB(4)]`` and the control is
    ``[collective_lift, pusher, p, q, r]``.  Collective and pusher commands are
    normalized.  Elevator trim can be supplied as an exogenous parameter for
    translational prediction; PX4 still moves the surfaces and realizes the
    requested body rates.
    """

    state_size = 10
    control_size = 5

    def __init__(self, plant: StandardVtolGazeboModel | None = None) -> None:
        self.plant = StandardVtolGazeboModel() if plant is None else plant

    def derivative(
        self,
        state: np.ndarray,
        control: np.ndarray,
        wind_w: np.ndarray | None = None,
        elevator_trim: float = 0.0,
        lift_weight: float = 1.0,
    ) -> np.ndarray:
        state = np.asarray(state, dtype=float)
        control = np.asarray(control, dtype=float)
        if state.shape != (self.state_size,):
            raise ValueError(f"state must have shape ({self.state_size},)")
        if control.shape != (self.control_size,):
            raise ValueError(f"control must have shape ({self.control_size},)")

        velocity_w = state[3:6]
        q_wb = state[6:10]
        body_rates = control[2:5]
        rotation_wb = quaternion_to_rotation(q_wb)
        velocity_b = rotation_wb.T @ velocity_w
        wind_w = np.zeros(3) if wind_w is None else np.asarray(wind_w, dtype=float)
        wind_b = rotation_wb.T @ wind_w

        lift_weight = float(np.clip(lift_weight, 0.0, 1.0))
        effective_lift = lift_weight * control[0]
        speeds = np.array(
            [self.plant.motors[index].target_speed(effective_lift) for index in range(4)]
            + [self.plant.motors[4].target_speed(control[1])]
        )
        motor_force_b, _ = self.plant.motor_wrench(speeds, velocity_b, body_rates, wind_b)
        aero_force_b, _ = self.plant.aerodynamic_wrench(
            np.array([0.0, 0.0, elevator_trim]),
            velocity_b,
            body_rates,
            wind_b,
        )

        result = np.zeros(self.state_size)
        result[0:3] = velocity_w
        result[3:6] = rotation_wb @ (motor_force_b + aero_force_b) / self.plant.mass + np.array(
            [0.0, 0.0, -self.plant.gravity]
        )
        result[6:10] = quaternion_derivative(q_wb, body_rates)
        return result

    def hover_state(self) -> np.ndarray:
        state = np.zeros(self.state_size)
        state[6] = 1.0
        return state

    def hover_control(self) -> np.ndarray:
        return np.array([self.plant.hover_command, 0.0, 0.0, 0.0, 0.0])
