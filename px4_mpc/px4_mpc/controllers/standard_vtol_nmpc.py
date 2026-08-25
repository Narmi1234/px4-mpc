"""acados NMPC for the reduced, PX4-rate-controlled Standard VTOL model."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import casadi as cs
import numpy as np

from px4_mpc.models.standard_vtol_casadi_model import (
    StandardVtolTransitionCasadiModel,
)


@dataclass(frozen=True)
class StandardVtolNmpcSolution:
    """One NMPC solution, including predictions and timing diagnostics."""

    status: int
    control: np.ndarray
    states: np.ndarray
    controls: np.ndarray
    solve_time: float


class StandardVtolNmpc:
    """Constrained 20 Hz transition NMPC with PX4 body-rate inner loop."""

    def __init__(
        self,
        horizon_steps: int = 30,
        horizon_seconds: float = 1.5,
        altitude_floor: float = -1000.0,
        build_directory: str | Path = "build/standard_vtol_nmpc",
        control_lower_bounds: np.ndarray | None = None,
        control_upper_bounds: np.ndarray | None = None,
    ) -> None:
        try:
            from acados_template import AcadosOcp, AcadosOcpSolver
        except ImportError as error:
            raise RuntimeError(
                "acados_template is unavailable; add "
                "acados/interfaces/acados_template to PYTHONPATH"
            ) from error

        self.N = int(horizon_steps)
        self.Tf = float(horizon_seconds)
        self.dt = self.Tf / self.N
        self.model = StandardVtolTransitionCasadiModel()
        self.build_directory = Path(build_directory).resolve()
        self.build_directory.mkdir(parents=True, exist_ok=True)

        ocp = AcadosOcp()
        model = self.model.get_acados_model()
        ocp.model = model
        ocp.solver_options.N_horizon = self.N
        nx = self.model.state_size
        nu = self.model.control_size

        # State/reference order: position, velocity, quaternion, commands.
        # Altitude, vertical speed, airspeed and attitude dominate the first
        # transition controller; absolute forward position is intentionally
        # assigned a low weight.
        q_state = np.array(
            [8.0, 8.0, 20.0, 12.0, 12.0, 60.0, 8.0, 35.0, 35.0, 20.0]
        )
        # Rate commands pass through PX4's inner loop. Penalize them strongly
        # enough that the outer NMPC does not chase small hover-attitude noise.
        r_control = np.array([40.0, 6.0, 20.0, 20.0, 15.0])
        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.model.cost_y_expr = cs.vertcat(model.x, model.u)
        ocp.model.cost_y_expr_e = model.x
        ocp.cost.W = np.diag(np.r_[q_state, r_control])
        ocp.cost.W_e = np.diag(4.0 * q_state)
        ocp.cost.yref = np.zeros(nx + nu)
        ocp.cost.yref_e = np.zeros(nx)

        default_lower = np.array([0.0, 0.0, -0.5, -0.5, -0.3])
        default_upper = np.array([0.65, 0.60, 0.5, 0.5, 0.3])
        lower = np.asarray(
            default_lower if control_lower_bounds is None else control_lower_bounds,
            dtype=float,
        )
        upper = np.asarray(
            default_upper if control_upper_bounds is None else control_upper_bounds,
            dtype=float,
        )
        if lower.shape != (nu,) or upper.shape != (nu,):
            raise ValueError("control bounds must match the five NMPC inputs")
        if not np.all(np.isfinite(lower)) or not np.all(lower < upper):
            raise ValueError("control bounds must be finite and strictly ordered")
        ocp.constraints.lbu = lower
        ocp.constraints.ubu = upper
        ocp.constraints.idxbu = np.arange(nu)
        ocp.constraints.idxbx = np.array([2])
        ocp.constraints.lbx = np.array([float(altitude_floor)])
        ocp.constraints.ubx = np.array([1000.0])
        ocp.constraints.x0 = np.r_[np.zeros(6), 1.0, np.zeros(3)]

        # In the normal transition envelope these expressions are sin(roll)
        # and sin(pitch). This avoids Euler-angle singularities in the OCP.
        qw, qx, qy, qz = (model.x[index] for index in range(6, 10))
        sin_roll = 2.0 * (qw * qx + qy * qz)
        sin_pitch = 2.0 * (qw * qy - qz * qx)
        ocp.model.con_h_expr = cs.vertcat(sin_roll, sin_pitch)
        ocp.constraints.lh = np.sin(np.deg2rad([-20.0, -20.0]))
        ocp.constraints.uh = np.sin(np.deg2rad([20.0, 15.0]))

        ocp.parameter_values = np.zeros(self.model.parameter_size)
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.sim_method_num_steps = 2
        ocp.solver_options.qp_solver_cond_N = self.N
        ocp.solver_options.tf = self.Tf
        ocp.code_export_directory = str(self.build_directory / "c_generated_code")
        json_path = self.build_directory / "acados_ocp.json"
        solver_library = (
            Path(ocp.code_export_directory)
            / f"libacados_ocp_solver_{model.name}.so"
        )
        model_directory = Path(__file__).resolve().parents[1] / "models"
        source_files = [
            Path(__file__).resolve(),
            model_directory / "standard_vtol_casadi_model.py",
            model_directory / "standard_vtol_gz_model.py",
        ]
        generated_is_current = (
            json_path.exists()
            and solver_library.exists()
            and solver_library.stat().st_mtime
            >= max(path.stat().st_mtime for path in source_files)
        )
        generate = not generated_is_current
        self.solver = AcadosOcpSolver(
            ocp,
            json_file=str(json_path),
            generate=generate,
            build=generate,
            verbose=generate,
        )

    def solve(
        self,
        state: np.ndarray,
        state_references: np.ndarray,
        control_references: np.ndarray,
        parameters: np.ndarray,
    ) -> StandardVtolNmpcSolution:
        """Solve one horizon and return the first command and prediction."""
        state = np.asarray(state, dtype=float).reshape(self.model.state_size)
        x_ref = np.asarray(state_references, dtype=float)
        u_ref = np.asarray(control_references, dtype=float)
        parameter_values = np.asarray(parameters, dtype=float)
        if x_ref.shape != (self.N + 1, self.model.state_size):
            raise ValueError("state_references has wrong shape")
        if u_ref.shape != (self.N, self.model.control_size):
            raise ValueError("control_references has wrong shape")
        if parameter_values.shape != (self.N + 1, self.model.parameter_size):
            raise ValueError("parameters has wrong shape")

        self.solver.set(0, "lbx", state)
        self.solver.set(0, "ubx", state)
        for stage in range(self.N):
            self.solver.set(stage, "yref", np.r_[x_ref[stage], u_ref[stage]])
            self.solver.set(stage, "p", parameter_values[stage])
            self.solver.set(stage, "x", x_ref[stage])
            self.solver.set(stage, "u", u_ref[stage])
        self.solver.set(self.N, "yref", x_ref[self.N])
        self.solver.set(self.N, "p", parameter_values[self.N])
        self.solver.set(self.N, "x", x_ref[self.N])

        status = int(self.solver.solve())
        states = np.vstack(
            [self.solver.get(stage, "x") for stage in range(self.N + 1)]
        )
        controls = np.vstack(
            [self.solver.get(stage, "u") for stage in range(self.N)]
        )
        solve_time = float(self.solver.get_stats("time_tot"))
        return StandardVtolNmpcSolution(
            status=status,
            control=controls[0].copy(),
            states=states,
            controls=controls,
            solve_time=solve_time,
        )
