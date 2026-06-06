"""Longitudinal standard VTOL model for offline NMPC experiments.

State:
    x = [px, pz, vx, vz, theta, q]

Controls:
    u = [lift_thrust, pusher_thrust, pitch_moment]

The model is intentionally compact. It captures the important transition
tradeoff for a standard VTOL quadplane: lift rotors support hover, the pusher
accelerates the aircraft, and aerodynamic lift grows with forward speed.
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
    iyy: float = 0.35
    gravity: float = 9.81
    rho: float = 1.225
    wing_area: float = 0.55
    cl_alpha: float = 4.2
    cl_max: float = 1.25
    cd0: float = 0.06
    induced_drag: float = 0.18
    max_lift_thrust: float = 70.0
    max_pusher_thrust: float = 30.0
    max_pitch_moment: float = 6.0
    min_speed_for_aero: float = 0.5


class StandardVtolModel:
    """Continuous-time dynamics and integration helpers."""

    state_names = ("px", "pz", "vx", "vz", "theta", "q")
    control_names = ("lift_thrust", "pusher_thrust", "pitch_moment")

    def __init__(self, params: StandardVtolParams | None = None):
        self.params = params or StandardVtolParams()

    @property
    def nx(self) -> int:
        return 6

    @property
    def nu(self) -> int:
        return 3

    @property
    def hover_control(self) -> np.ndarray:
        p = self.params
        return np.array([p.mass * p.gravity, 0.0, 0.0], dtype=float)

    @property
    def control_lower_bounds(self) -> np.ndarray:
        p = self.params
        return np.array([0.0, 0.0, -p.max_pitch_moment], dtype=float)

    @property
    def control_upper_bounds(self) -> np.ndarray:
        p = self.params
        return np.array(
            [p.max_lift_thrust, p.max_pusher_thrust, p.max_pitch_moment],
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
        return ca.Function("standard_vtol_f", [x, u], [xdot], ["x", "u"], ["xdot"])

    def dynamics(self, x, u, symbolic: bool = False):
        """Return continuous-time dynamics for numeric numpy or CasADi values."""

        p = self.params
        lib = ca if symbolic else np
        if symbolic:
            self._require_casadi()

        px, pz, vx, vz, theta, q = x[0], x[1], x[2], x[3], x[4], x[5]
        lift_thrust, pusher_thrust, pitch_moment = u[0], u[1], u[2]

        speed_sq = vx * vx + vz * vz
        if symbolic:
            speed = ca.sqrt(speed_sq + p.min_speed_for_aero**2)
            gamma = ca.atan2(vz, vx + 1e-3)
            alpha = theta - gamma
            cl = p.cl_max * ca.tanh((p.cl_alpha / p.cl_max) * alpha)
        else:
            speed = float(np.sqrt(speed_sq + p.min_speed_for_aero**2))
            gamma = float(np.arctan2(vz, vx + 1e-3))
            alpha = float(theta - gamma)
            cl = float(p.cl_max * np.tanh((p.cl_alpha / p.cl_max) * alpha))

        cd = p.cd0 + p.induced_drag * cl * cl
        qbar = 0.5 * p.rho * speed * speed
        aero_lift = qbar * p.wing_area * cl
        aero_drag = qbar * p.wing_area * cd

        evx = vx / speed
        evz = vz / speed
        lift_dir_x = -evz
        lift_dir_z = evx

        pusher_x = pusher_thrust * lib.cos(theta)
        pusher_z = pusher_thrust * lib.sin(theta)

        rotor_x = -lift_thrust * lib.sin(theta)
        rotor_z = lift_thrust * lib.cos(theta)

        drag_x = -aero_drag * evx
        drag_z = -aero_drag * evz

        wing_x = aero_lift * lift_dir_x
        wing_z = aero_lift * lift_dir_z

        force_x = pusher_x + rotor_x + drag_x + wing_x
        force_z = pusher_z + rotor_z + drag_z + wing_z - p.mass * p.gravity

        return lib.vertcat(
            vx,
            vz,
            force_x / p.mass,
            force_z / p.mass,
            q,
            pitch_moment / p.iyy,
        ) if symbolic else np.array(
            [
                vx,
                vz,
                force_x / p.mass,
                force_z / p.mass,
                q,
                pitch_moment / p.iyy,
            ],
            dtype=float,
        )

    def rk4_step(self, x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
        k1 = self.dynamics(x, u)
        k2 = self.dynamics(x + 0.5 * dt * k1, u)
        k3 = self.dynamics(x + 0.5 * dt * k2, u)
        k4 = self.dynamics(x + dt * k3, u)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def _require_casadi(self) -> None:
        if ca is None:
            raise ModuleNotFoundError(
                "CasADi is required for NMPC. Install with: "
                "python3 -m pip install -r standard_vtol_nmpc/requirements.txt"
            )

