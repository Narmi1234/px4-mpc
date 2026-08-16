"""MPC controller implementations."""

try:
    from px4_mpc.controllers.standard_vtol_nmpc import StandardVtolNmpc
except (ImportError, RuntimeError):
    StandardVtolNmpc = None

__all__ = ["StandardVtolNmpc"]
