"""6DOF standard VTOL model for offline NMPC experiments.

State:
    x = [px, py, pz, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz]

Controls:
    u = [lift_thrust, pusher_thrust, roll_moment, pitch_moment, yaw_moment]

The model is intentionally compact, but it is a true rigid-body 6DOF model:
position and velocity are expressed in the world frame, orientation is a unit
quaternion, and angular velocity is expressed in the body frame.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

try:
    import casadi as ca
except ModuleNotFoundError:  # pragma: no cover - handled at runtime by callers
    ca = None


@dataclass(frozen=True)
class StandardVtolParams:
    mass: float = 4.0
    ixx: float = 0.24
    iyy: float = 0.35
    izz: float = 0.48
    gravity: float = 9.81
    rho: float = 1.225
    wing_area: float = 0.55
    cl_alpha: float = 4.2
    cl_max: float = 1.25
    cd0: float = 0.06
    induced_drag: float = 0.18
    cy_beta: float = 0.35
    angular_damping_x: float = 0.08
    angular_damping_y: float = 0.10
    angular_damping_z: float = 0.06
    max_lift_thrust: float = 70.0
    max_pusher_thrust: float = 30.0
    max_roll_moment: float = 4.5
    max_pitch_moment: float = 6.0
    max_yaw_moment: float = 3.0
    min_speed_for_aero: float = 0.5


class StandardVtolModel:
    """Continuous-time 6DOF dynamics and integration helpers."""

    state_names = (
        "px",
        "py",
        "pz",
        "vx",
        "vy",
        "vz",
        "qw",
        "qx",
        "qy",
        "qz",
        "wx",
        "wy",
        "wz",
    )
    control_names = (
        "lift_thrust",
        "pusher_thrust",
        "roll_moment",
        "pitch_moment",
        "yaw_moment",
    )

    def __init__(self, params: StandardVtolParams | None = None):
        self.params = params or StandardVtolParams()

    @property
    def nx(self) -> int:
        return 13

    @property
    def nu(self) -> int:
        return 5

    @property
    def hover_control(self) -> np.ndarray:
        p = self.params
        return np.array([p.mass * p.gravity, 0.0, 0.0, 0.0, 0.0], dtype=float)

    @property
    def forward_trim_control(self) -> np.ndarray:
        return np.array([12.0, 8.0, 0.0, 0.0, 0.0], dtype=float)

    @property
    def control_lower_bounds(self) -> np.ndarray:
        p = self.params
        return np.array(
            [
                0.0,
                0.0,
                -p.max_roll_moment,
                -p.max_pitch_moment,
                -p.max_yaw_moment,
            ],
            dtype=float,
        )

    @property
    def control_upper_bounds(self) -> np.ndarray:
        p = self.params
        return np.array(
            [
                p.max_lift_thrust,
                p.max_pusher_thrust,
                p.max_roll_moment,
                p.max_pitch_moment,
                p.max_yaw_moment,
            ],
            dtype=float,
        )

    @property
    def inertia(self) -> np.ndarray:
        p = self.params
        return np.array([p.ixx, p.iyy, p.izz], dtype=float)

    @property
    def angular_damping(self) -> np.ndarray:
        p = self.params
        return np.array(
            [p.angular_damping_x, p.angular_damping_y, p.angular_damping_z],
            dtype=float,
        )

    def casadi_symbols(self):
        self._require_casadi()
        x = ca.SX.sym("x", self.nx)
        u = ca.SX.sym("u", self.nu)
        return x, u

    def casadi_dynamics(self):
        self._require_casadi()
        x, u = self.casadi_symbols()
        xdot = self.dynamics(x, u, symbolic=True)
        return ca.Function("standard_vtol_6dof_f", [x, u], [xdot], ["x", "u"], ["xdot"])

    def dynamics(self, x, u, symbolic: bool = False):
        """Return continuous-time 6DOF dynamics for numpy or CasADi values."""

        p = self.params
        if symbolic:
            self._require_casadi()

        vel = x[3:6]
        quat = self.quaternion_normalize(x[6:10], symbolic=symbolic)
        omega = x[10:13]

        lift_thrust = u[0]
        pusher_thrust = u[1]
        moments_cmd = u[2:5]

        rot_body_to_world = self.quaternion_to_rotation_matrix(quat, symbolic=symbolic)
        vel_body = self._mtimes(rot_body_to_world.T, vel, symbolic=symbolic)

        aero_body = self._aerodynamic_force_body(vel_body, symbolic=symbolic)
        thrust_body = self._vector(
            pusher_thrust,
            0.0,
            lift_thrust,
            symbolic=symbolic,
        )
        force_body = thrust_body + aero_body
        force_world = self._mtimes(rot_body_to_world, force_body, symbolic=symbolic)
        gravity_world = self._vector(
            0.0,
            0.0,
            -p.mass * p.gravity,
            symbolic=symbolic,
        )

        pos_dot = vel
        vel_dot = (force_world + gravity_world) / p.mass
        quat_dot = 0.5 * self._mtimes(
            self.quaternion_omega_matrix(omega, symbolic=symbolic),
            quat,
            symbolic=symbolic,
        )
        omega_dot = self._angular_acceleration(
            omega,
            moments_cmd,
            symbolic=symbolic,
        )

        if symbolic:
            return ca.vertcat(pos_dot, vel_dot, quat_dot, omega_dot)

        return np.concatenate(
            [
                np.asarray(pos_dot, dtype=float),
                np.asarray(vel_dot, dtype=float),
                np.asarray(quat_dot, dtype=float),
                np.asarray(omega_dot, dtype=float),
            ]
        )

    def rk4_step(self, x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
        k1 = self.dynamics(x, u)
        k2 = self.dynamics(x + 0.5 * dt * k1, u)
        k3 = self.dynamics(x + 0.5 * dt * k2, u)
        k4 = self.dynamics(x + dt * k3, u)
        x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        return self.normalize_state(x_next)

    def normalize_state(self, x: np.ndarray) -> np.ndarray:
        x_norm = np.array(x, dtype=float, copy=True)
        x_norm[6:10] = self.quaternion_normalize(x_norm[6:10])
        return x_norm

    def interpolate_state(self, x0: np.ndarray, x1: np.ndarray, progress: float) -> np.ndarray:
        state = (1.0 - progress) * x0 + progress * x1
        state[6:10] = self.interpolate_quaternion(x0[6:10], x1[6:10], progress)
        return state

    def interpolate_quaternion(
        self,
        q0: np.ndarray,
        q1: np.ndarray,
        progress: float,
    ) -> np.ndarray:
        q0_norm = self.quaternion_normalize(q0)
        q1_norm = self.quaternion_normalize(q1)
        if float(np.dot(q0_norm, q1_norm)) < 0.0:
            q1_norm = -q1_norm
        q = (1.0 - progress) * q0_norm + progress * q1_norm
        return self.quaternion_normalize(q)

    def quaternion_normalize(self, q, symbolic: bool = False):
        if symbolic:
            self._require_casadi()
            return q / ca.sqrt(ca.sumsqr(q) + 1e-12)

        q_arr = np.asarray(q, dtype=float)
        norm = float(np.linalg.norm(q_arr))
        if norm < 1e-12:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        q_norm = q_arr / norm
        if q_norm[0] < 0.0:
            q_norm = -q_norm
        return q_norm

    def quaternion_to_rotation_matrix(self, q, symbolic: bool = False):
        qw, qx, qy, qz = q[0], q[1], q[2], q[3]
        rows = (
            (
                1 - 2 * (qy * qy + qz * qz),
                2 * (qx * qy - qw * qz),
                2 * (qx * qz + qw * qy),
            ),
            (
                2 * (qx * qy + qw * qz),
                1 - 2 * (qx * qx + qz * qz),
                2 * (qy * qz - qw * qx),
            ),
            (
                2 * (qx * qz - qw * qy),
                2 * (qy * qz + qw * qx),
                1 - 2 * (qx * qx + qy * qy),
            ),
        )
        if symbolic:
            self._require_casadi()
            return ca.vertcat(*(ca.horzcat(*row) for row in rows))
        return np.array(rows, dtype=float)

    def quaternion_omega_matrix(self, omega, symbolic: bool = False):
        wx, wy, wz = omega[0], omega[1], omega[2]
        rows = (
            (0.0, -wx, -wy, -wz),
            (wx, 0.0, wz, -wy),
            (wy, -wz, 0.0, wx),
            (wz, wy, -wx, 0.0),
        )
        if symbolic:
            self._require_casadi()
            return ca.vertcat(*(ca.horzcat(*row) for row in rows))
        return np.array(rows, dtype=float)

    def quaternion_to_euler(self, q: np.ndarray) -> np.ndarray:
        """Return roll, pitch, yaw in radians from a [qw, qx, qy, qz] quaternion."""

        rot = self.quaternion_to_rotation_matrix(self.quaternion_normalize(q))
        roll = np.arctan2(rot[2, 1], rot[2, 2])
        pitch = np.arcsin(np.clip(-rot[2, 0], -1.0, 1.0))
        yaw = np.arctan2(rot[1, 0], rot[0, 0])
        return np.array([roll, pitch, yaw], dtype=float)

    def _aerodynamic_force_body(self, vel_body, symbolic: bool = False):
        p = self.params
        if symbolic:
            speed = ca.sqrt(ca.sumsqr(vel_body) + p.min_speed_for_aero**2)
            alpha = -ca.atan2(vel_body[2], vel_body[0] + 1e-3)
            longitudinal_speed = ca.sqrt(
                vel_body[0] * vel_body[0]
                + vel_body[2] * vel_body[2]
                + p.min_speed_for_aero**2
            )
            beta = ca.atan2(
                vel_body[1],
                longitudinal_speed,
            )
            cl = p.cl_max * ca.tanh((p.cl_alpha / p.cl_max) * alpha)
            sin_alpha = ca.sin(alpha)
            cos_alpha = ca.cos(alpha)
        else:
            speed = float(np.sqrt(np.dot(vel_body, vel_body) + p.min_speed_for_aero**2))
            alpha = float(-np.arctan2(vel_body[2], vel_body[0] + 1e-3))
            longitudinal_speed = float(
                np.sqrt(
                    vel_body[0] * vel_body[0]
                    + vel_body[2] * vel_body[2]
                    + p.min_speed_for_aero**2
                )
            )
            beta = float(
                np.arctan2(
                    vel_body[1],
                    longitudinal_speed,
                )
            )
            cl = float(p.cl_max * np.tanh((p.cl_alpha / p.cl_max) * alpha))
            sin_alpha = float(np.sin(alpha))
            cos_alpha = float(np.cos(alpha))

        cd = p.cd0 + p.induced_drag * cl * cl
        qbar = 0.5 * p.rho * speed * speed
        lift = qbar * p.wing_area * cl
        drag = qbar * p.wing_area * cd
        side_force = -qbar * p.wing_area * p.cy_beta * beta

        lift_body = self._vector(
            -lift * sin_alpha,
            side_force,
            lift * cos_alpha,
            symbolic=symbolic,
        )
        drag_body = -(drag / speed) * vel_body
        return lift_body + drag_body

    def _angular_acceleration(self, omega, moments_cmd, symbolic: bool = False):
        inertia = self.inertia
        damping = self.angular_damping
        damped_moments = moments_cmd - self._vector(
            damping[0] * omega[0],
            damping[1] * omega[1],
            damping[2] * omega[2],
            symbolic=symbolic,
        )
        inertia_omega = self._vector(
            inertia[0] * omega[0],
            inertia[1] * omega[1],
            inertia[2] * omega[2],
            symbolic=symbolic,
        )
        gyroscopic = self._cross(omega, inertia_omega, symbolic=symbolic)
        net_moments = damped_moments - gyroscopic
        return self._vector(
            net_moments[0] / inertia[0],
            net_moments[1] / inertia[1],
            net_moments[2] / inertia[2],
            symbolic=symbolic,
        )

    def _vector(self, x, y, z, symbolic: bool = False):
        if symbolic:
            self._require_casadi()
            return ca.vertcat(x, y, z)
        return np.array([x, y, z], dtype=float)

    def _cross(self, a, b, symbolic: bool = False):
        if symbolic:
            self._require_casadi()
            return ca.vertcat(
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0],
            )
        return np.cross(a, b)

    def _mtimes(self, a, b, symbolic: bool = False):
        if symbolic:
            self._require_casadi()
            return ca.mtimes(a, b)
        return a @ b

    def _require_casadi(self) -> None:
        if ca is None:
            raise ModuleNotFoundError(
                "CasADi is required for NMPC. Install with: "
                "python3 -m pip install -r standard_vtol_nmpc/requirements.txt"
            )
