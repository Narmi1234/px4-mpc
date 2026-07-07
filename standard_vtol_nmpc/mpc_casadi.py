"""CasADi/Ipopt NMPC formulation for a 6DOF standard VTOL transition."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

try:
    import casadi as ca
except ModuleNotFoundError:  # pragma: no cover - handled when constructing MPC
    ca = None

try:
    from .model import StandardVtolModel
except ImportError:  # Allows: python standard_vtol_nmpc/simulate.py
    from model import StandardVtolModel


@dataclass(frozen=True)
class MpcConfig:
    dt: float = 0.15
    horizon_steps: int = 18
    max_ipopt_iter: int = 160
    max_tilt_deg: float = 70.0
    max_body_rate: float = 2.0
    enforce_state_constraints: bool = True
    integration_method: str = "rk4"
    moment_weight_scale: float = 1.0


class StandardVtolNMPC:
    """Multiple-shooting NMPC solver built with CasADi Opti."""

    def __init__(self, model: StandardVtolModel, config: MpcConfig | None = None):
        if ca is None:
            raise ModuleNotFoundError(
                "CasADi is required for NMPC. Install with: "
                "python3 -m pip install -r standard_vtol_nmpc/requirements.txt"
            )

        self.model = model
        self.config = config or MpcConfig()
        self._build_problem()

    def solve(
        self,
        x0: np.ndarray,
        x_ref: np.ndarray,
        previous_solution: dict[str, np.ndarray] | None = None,
    ) -> dict[str, np.ndarray | float]:
        x0 = self.model.normalize_state(x0)
        x_ref = self.model.normalize_state(x_ref)

        opti = self.opti
        opti.set_value(self.x0_param, x0)
        opti.set_value(self.x_ref_param, x_ref)

        if previous_solution is None:
            self._set_initial_guess(x0, x_ref)
        else:
            self._warm_start(previous_solution)

        try:
            sol = opti.solve()
            x_pred = np.array(sol.value(self.X), dtype=float)
            u_pred = np.array(sol.value(self.U), dtype=float)
            objective = float(sol.value(self.objective))
            status = "Solve_Succeeded"
        except RuntimeError:
            stats = opti.debug
            x_pred = np.array(stats.value(self.X), dtype=float)
            u_pred = np.array(stats.value(self.U), dtype=float)
            objective = float(stats.value(self.objective))
            status = self._return_status("Solver_Failed_Debug_Solution")

        for k in range(x_pred.shape[1]):
            x_pred[:, k] = self.model.normalize_state(x_pred[:, k])

        return {
            "x_pred": x_pred,
            "u_pred": u_pred,
            "u0": u_pred[:, 0].copy(),
            "objective": objective,
            "status": status,
        }

    def _build_problem(self) -> None:
        cfg = self.config
        nx = self.model.nx
        nu = self.model.nu
        n = cfg.horizon_steps

        opti = ca.Opti()
        X = opti.variable(nx, n + 1)
        U = opti.variable(nu, n)
        x0_param = opti.parameter(nx)
        x_ref_param = opti.parameter(nx)

        f = self.model.casadi_dynamics()
        objective = 0.0

        weights = {
            "position": np.diag([0.18, 5.0, 18.0]),
            "velocity": np.diag([3.0, 2.0, 4.0]),
            "attitude": np.diag([12.0, 14.0, 5.0]),
            "omega": np.diag([0.4, 0.5, 0.25]),
            "quat_norm": 20.0,
        }
        terminal_weights = {
            "position": np.diag([0.7, 20.0, 65.0]),
            "velocity": np.diag([12.0, 6.0, 14.0]),
            "attitude": np.diag([35.0, 45.0, 15.0]),
            "omega": np.diag([1.5, 1.8, 1.0]),
            "quat_norm": 60.0,
        }
        r_weights = np.diag([0.012, 0.035, 0.09, 0.11, 0.08])
        du_weights = np.diag([0.05, 0.03, 0.14, 0.16, 0.11])
        moment_weight_scale = max(float(cfg.moment_weight_scale), 0.0)
        r_weights[2:5, 2:5] *= moment_weight_scale
        du_weights[2:5, 2:5] *= moment_weight_scale

        hover_trim_u = ca.DM(self.model.hover_control)

        opti.subject_to(X[:, 0] == x0_param)
        for k in range(n):
            x_next = self._integrate_symbolic(f, X[:, k], U[:, k], cfg.dt)
            opti.subject_to(X[:, k + 1] == x_next)

            progress = (k + 1) / n
            scheduled_ref = self._scheduled_reference(x0_param, x_ref_param, progress)
            objective += self._state_tracking_cost(X[:, k], scheduled_ref, weights)

            control_error = U[:, k] - hover_trim_u
            objective += ca.mtimes([control_error.T, r_weights, control_error])
            if k > 0:
                delta_u = U[:, k] - U[:, k - 1]
                objective += ca.mtimes([delta_u.T, du_weights, delta_u])

        objective += self._state_tracking_cost(X[:, n], x_ref_param, terminal_weights)

        lb_u = self.model.control_lower_bounds
        ub_u = self.model.control_upper_bounds
        for idx in range(nu):
            opti.subject_to(opti.bounded(lb_u[idx], U[idx, :], ub_u[idx]))

        if cfg.enforce_state_constraints:
            min_body_z_world_z = np.cos(np.deg2rad(cfg.max_tilt_deg))
            for k in range(n + 1):
                quat = X[6:10, k]
                opti.subject_to(opti.bounded(0.95, ca.sumsqr(quat), 1.05))
                opti.subject_to(quat[0] >= 0.0)
                quat_unit = self._quat_normalize(quat)
                body_z_world_z = 1.0 - 2.0 * (
                    quat_unit[1] * quat_unit[1] + quat_unit[2] * quat_unit[2]
                )
                opti.subject_to(body_z_world_z >= min_body_z_world_z)

            opti.subject_to(opti.bounded(-8.0, X[4, 1:], 8.0))
            opti.subject_to(opti.bounded(-6.0, X[5, 1:], 6.0))
            for idx in range(10, 13):
                opti.subject_to(
                    opti.bounded(
                        -cfg.max_body_rate,
                        X[idx, 1:],
                        cfg.max_body_rate,
                    )
                )

        opti.minimize(objective)
        opts = {
            "expand": True,
            "print_time": False,
            "ipopt.print_level": 0,
            "ipopt.max_iter": cfg.max_ipopt_iter,
            "ipopt.tol": 1e-4,
            "ipopt.acceptable_tol": 1e-3,
            "ipopt.hessian_approximation": "limited-memory",
            "ipopt.mu_strategy": "adaptive",
        }
        opti.solver("ipopt", opts)

        self.opti = opti
        self.X = X
        self.U = U
        self.x0_param = x0_param
        self.x_ref_param = x_ref_param
        self.objective = objective

    def _return_status(self, fallback: str) -> str:
        try:
            return str(self.opti.stats().get("return_status", fallback))
        except RuntimeError:
            return fallback

    def _scheduled_reference(self, x0, x_ref, progress: float):
        horizon_duration = self.config.dt * self.config.horizon_steps
        elapsed = progress * horizon_duration
        # Integrate the linearly scheduled forward speed to obtain px_ref.
        acceleration_x = (x_ref[3] - x0[3]) / horizon_duration
        px_ref = x0[0] + x0[3] * elapsed + 0.5 * acceleration_x * elapsed * elapsed
        yz_ref = x0[1:3] + progress * (x_ref[1:3] - x0[1:3])
        pos_ref = ca.vertcat(px_ref, yz_ref)
        vel_ref = x0[3:6] + progress * (x_ref[3:6] - x0[3:6])
        return ca.vertcat(pos_ref, vel_ref, x_ref[6:10], x_ref[10:13])

    def _state_tracking_cost(self, x, x_ref, weights: dict[str, np.ndarray]):
        pos_error = x[0:3] - x_ref[0:3]
        vel_error = x[3:6] - x_ref[3:6]
        quat_error = self._quat_error_vector(x[6:10], x_ref[6:10])
        omega_error = x[10:13] - x_ref[10:13]

        cost = ca.mtimes([pos_error.T, weights["position"], pos_error])
        cost += ca.mtimes([vel_error.T, weights["velocity"], vel_error])
        cost += ca.mtimes([quat_error.T, weights["attitude"], quat_error])
        cost += ca.mtimes([omega_error.T, weights["omega"], omega_error])
        quat_norm_error = ca.sumsqr(x[6:10]) - 1.0
        cost += weights["quat_norm"] * quat_norm_error * quat_norm_error
        return cost

    def _quat_error_vector(self, q, q_ref):
        q = self._quat_normalize(q)
        q_ref = self._quat_normalize(q_ref)
        q_ref_conj = ca.vertcat(q_ref[0], -q_ref[1], -q_ref[2], -q_ref[3])
        q_err = self._quat_multiply(q_ref_conj, q)
        return 2.0 * q_err[1:4]

    def _quat_normalize(self, q):
        return q / ca.sqrt(ca.sumsqr(q) + 1e-12)

    def _quat_multiply(self, a, b):
        aw, ax, ay, az = a[0], a[1], a[2], a[3]
        bw, bx, by, bz = b[0], b[1], b[2], b[3]
        return ca.vertcat(
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        )

    def _rk4_symbolic(self, f, x, u, dt: float):
        k1 = f(x=x, u=u)["xdot"]
        k2 = f(x=x + 0.5 * dt * k1, u=u)["xdot"]
        k3 = f(x=x + 0.5 * dt * k2, u=u)["xdot"]
        k4 = f(x=x + dt * k3, u=u)["xdot"]
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def _euler_symbolic(self, f, x, u, dt: float):
        return x + dt * f(x=x, u=u)["xdot"]

    def _integrate_symbolic(self, f, x, u, dt: float):
        method = self.config.integration_method.strip().lower()
        if method == "euler":
            return self._euler_symbolic(f, x, u, dt)
        if method == "rk4":
            return self._rk4_symbolic(f, x, u, dt)
        raise ValueError(f"Unsupported NMPC integration method: {method}")

    def _set_initial_guess(self, x0: np.ndarray, x_ref: np.ndarray) -> None:
        n = self.config.horizon_steps
        for k in range(n + 1):
            progress = k / n
            guess_x = self.model.interpolate_state(x0, x_ref, progress)
            self.opti.set_initial(self.X[:, k], guess_x)
        for k in range(n):
            self.opti.set_initial(self.U[:, k], self.model.hover_control)

    def _warm_start(self, previous_solution: dict[str, np.ndarray]) -> None:
        x_prev = previous_solution["x_pred"]
        u_prev = previous_solution["u_pred"]
        self.opti.set_initial(self.X[:, :-1], x_prev[:, 1:])
        self.opti.set_initial(self.X[:, -1], x_prev[:, -1])
        self.opti.set_initial(self.U[:, :-1], u_prev[:, 1:])
        self.opti.set_initial(self.U[:, -1], u_prev[:, -1])
