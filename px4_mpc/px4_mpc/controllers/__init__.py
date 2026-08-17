"""MPC controller implementations."""

from px4_mpc.controllers.standard_vtol_output import (
    limit_external_pusher_command,
    limit_mc_command,
    vertical_hover_lift,
)

try:
    from px4_mpc.controllers.standard_vtol_nmpc import StandardVtolNmpc
except (ImportError, RuntimeError):
    StandardVtolNmpc = None

__all__ = [
    "StandardVtolNmpc",
    "limit_external_pusher_command",
    "limit_mc_command",
    "vertical_hover_lift",
]
