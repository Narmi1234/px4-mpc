"""CasADi/Ipopt NMPC formulation for the standard VTOL transition."""

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
    dt: float = 0.12
    horizon_steps: int = 25
    max_ipopt_iter: int = 120


class StandardVtolNMPC:
    """Small multiple-shooting NMPC solver built with CasADi Opti."""

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
            status = "Solver_Failed_Debug_Solution"

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

        q_weights = np.diag([0.2, 18.0, 3.0, 4.0, 8.0, 0.4])
        q_terminal = np.diag([0.5, 50.0, 12.0, 8.0, 18.0, 1.0])
        r_weights = np.diag([0.012, 0.035, 0.08])
        du_weights = np.diag([0.05, 0.03, 0.12])

        hover_u = self.model.hover_control
        forward_trim_u = np.array([12.0, 8.0, 0.0])
        u_ref = opti.parameter(nu)
        opti.set_value(u_ref, forward_trim_u)

        opti.subject_to(X[:, 0] == x0_param)
        for k in range(n):
            x_next = self._rk4_symbolic(f, X[:, k], U[:, k], cfg.dt)
            opti.subject_to(X[:, k + 1] == x_next)

            progress = (k + 1) / n
            scheduled_ref = x0_param + progress * (x_ref_param - x0_param)
            state_error = X[:, k] - scheduled_ref
            control_error = U[:, k] - ((1.0 - progress) * hover_u + progress * u_ref)

            objective += ca.mtimes([state_error.T, q_weights, state_error])
            objective += ca.mtimes([control_error.T, r_weights, control_error])
            if k > 0:
                delta_u = U[:, k] - U[:, k - 1]
                objective += ca.mtimes([delta_u.T, du_weights, delta_u])

        terminal_error = X[:, n] - x_ref_param
        objective += ca.mtimes([terminal_error.T, q_terminal, terminal_error])

        lb_u = self.model.control_lower_bounds
        ub_u = self.model.control_upper_bounds
        opti.subject_to(opti.bounded(lb_u[0], U[0, :], ub_u[0]))
        opti.subject_to(opti.bounded(lb_u[1], U[1, :], ub_u[1]))
        opti.subject_to(opti.bounded(lb_u[2], U[2, :], ub_u[2]))

        opti.subject_to(opti.bounded(-0.45, X[4, :], 0.45))
        opti.subject_to(opti.bounded(-1.5, X[5, :], 1.5))
        opti.subject_to(opti.bounded(-6.0, X[3, :], 6.0))

        opti.minimize(objective)
        opts = {
            "expand": True,
            "print_time": False,
            "ipopt.print_level": 0,
            "ipopt.max_iter": cfg.max_ipopt_iter,
            "ipopt.tol": 1e-4,
            "ipopt.acceptable_tol": 1e-3,
        }
        opti.solver("ipopt", opts)

        self.opti = opti
        self.X = X
        self.U = U
        self.x0_param = x0_param
        self.x_ref_param = x_ref_param
        self.objective = objective

    def _rk4_symbolic(self, f, x, u, dt: float):
        k1 = f(x=x, u=u)["xdot"]
        k2 = f(x=x + 0.5 * dt * k1, u=u)["xdot"]
        k3 = f(x=x + 0.5 * dt * k2, u=u)["xdot"]
        k4 = f(x=x + dt * k3, u=u)["xdot"]
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def _set_initial_guess(self, x0: np.ndarray, x_ref: np.ndarray) -> None:
        n = self.config.horizon_steps
        for k in range(n + 1):
            progress = k / n
            self.opti.set_initial(self.X[:, k], x0 + progress * (x_ref - x0))
        for k in range(n):
            progress = (k + 1) / n
            guess_u = (1.0 - progress) * self.model.hover_control + progress * np.array(
                [12.0, 8.0, 0.0]
            )
            self.opti.set_initial(self.U[:, k], guess_u)

    def _warm_start(self, previous_solution: dict[str, np.ndarray]) -> None:
        x_prev = previous_solution["x_pred"]
        u_prev = previous_solution["u_pred"]
        self.opti.set_initial(self.X[:, :-1], x_prev[:, 1:])
        self.opti.set_initial(self.X[:, -1], x_prev[:, -1])
        self.opti.set_initial(self.U[:, :-1], u_prev[:, 1:])
        self.opti.set_initial(self.U[:, -1], u_prev[:, -1])
