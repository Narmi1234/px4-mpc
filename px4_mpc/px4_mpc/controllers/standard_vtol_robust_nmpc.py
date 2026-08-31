"""acados NMPC for the 16-state robust Standard VTOL transition model."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import casadi as cs
import numpy as np

from px4_mpc.models.standard_vtol_robust_casadi_model import (
    StandardVtolRobustCasadiModel,
)


@dataclass(frozen=True)
class StandardVtolRobustNmpcSolution:
    """One robust-model OCP solution and its prediction."""

    status: int
    control: np.ndarray
    states: np.ndarray
    controls: np.ndarray
    solve_time: float


class StandardVtolRobustNmpc:
    """Constrained NMPC that owns lift/pusher allocation during transition.

    The sixth input, ``lambda``, is the lift-motor effectiveness fraction:
    one is multicopter flight and zero is fixed-wing flight.  The first
    implementation intentionally remains a nominal OCP. Robustness is checked
    by replaying the resulting closed loop with bounded model disturbances.
    """

    def __init__(
        self,
        horizon_steps: int = 25,
        horizon_seconds: float = 2.0,
        altitude_floor: float = -1000.0,
        build_directory: str | Path = "build/standard_vtol_robust_nmpc",
    ) -> None:
        try:
            from acados_template import AcadosOcp, AcadosOcpSolver
        except ImportError as error:
            raise RuntimeError(
                "acados_template is unavailable; source "
                "scripts/setup_standard_vtol_nmpc.bash"
            ) from error

        self.N = int(horizon_steps)
        self.Tf = float(horizon_seconds)
        if self.N < 2 or self.Tf <= 0.0:
            raise ValueError("horizon must contain at least two positive stages")
        self.dt = self.Tf / self.N
        self.model = StandardVtolRobustCasadiModel()
        self.build_directory = Path(build_directory).resolve()
        self.build_directory.mkdir(parents=True, exist_ok=True)

        ocp = AcadosOcp()
        model = self.model.get_acados_model()
        ocp.model = model
        ocp.solver_options.N_horizon = self.N
        nx = self.model.state_size
        nu = self.model.control_size

        # [position, velocity, quaternion, body rates, surface angles].
        # Forward position is deliberately light; altitude, forward speed,
        # pitch attitude and pitch rate dominate the transition corridor.
        q_state = np.array(
            [
                0.15, 4.0, 35.0,
                18.0, 12.0, 70.0,
                8.0, 25.0, 45.0, 18.0,
                8.0, 24.0, 7.0,
                0.8, 0.8, 1.5,
            ]
        )
        # [collective, pusher, p_sp, q_sp, r_sp, lambda]. Smooth lambda
        # references supply the transition schedule until lambda is promoted
        # to a state with a hard slew constraint in the live controller.
        r_control = np.array([28.0, 7.0, 15.0, 22.0, 12.0, 18.0])
        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.model.cost_y_expr = cs.vertcat(model.x, model.u)
        ocp.model.cost_y_expr_e = model.x
        ocp.cost.W = np.diag(np.r_[q_state, r_control])
        ocp.cost.W_e = np.diag(4.0 * q_state)
        ocp.cost.yref = np.zeros(nx + nu)
        ocp.cost.yref_e = np.zeros(nx)

        ocp.constraints.idxbu = np.arange(nu)
        ocp.constraints.lbu = np.array(
            [0.0, 0.0, -0.45, -0.45, -0.30, 0.0]
        )
        ocp.constraints.ubu = np.array(
            [0.70, 0.70, 0.45, 0.45, 0.30, 1.0]
        )

        # Box constraints protect vertical speed, body rates and surfaces.
        ocp.constraints.idxbx = np.array([2, 5, 10, 11, 12, 13, 14, 15])
        ocp.constraints.lbx = np.array(
            [altitude_floor, -2.0, -0.65, -0.65, -0.45, -0.70, -0.70, -0.70]
        )
        ocp.constraints.ubx = np.array(
            [1000.0, 2.0, 0.65, 0.65, 0.45, 0.70, 0.70, 0.70]
        )
        ocp.constraints.x0 = self.model.hover_state()

        # These equal sin(roll) and sin(pitch) in the intended envelope.
        qw, qx, qy, qz = (model.x[index] for index in range(6, 10))
        sin_roll = 2.0 * (qw * qx + qy * qz)
        sin_pitch = 2.0 * (qw * qy - qz * qx)
        ocp.model.con_h_expr = cs.vertcat(sin_roll, sin_pitch)
        ocp.constraints.lh = np.sin(np.deg2rad([-22.0, -22.0]))
        ocp.constraints.uh = np.sin(np.deg2rad([22.0, 18.0]))

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
        source_files = [
            Path(__file__).resolve(),
            Path(__file__).resolve().parents[1]
            / "models"
            / "standard_vtol_robust_casadi_model.py",
            Path(__file__).resolve().parents[1]
            / "models"
            / "standard_vtol_pitch_rate_model.py",
            Path(__file__).resolve().parents[1]
            / "models"
            / "standard_vtol_gz_model.py",
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
    ) -> StandardVtolRobustNmpcSolution:
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
        return StandardVtolRobustNmpcSolution(
            status=status,
            control=controls[0].copy(),
            states=states,
            controls=controls,
            solve_time=float(self.solver.get_stats("time_tot")),
        )
