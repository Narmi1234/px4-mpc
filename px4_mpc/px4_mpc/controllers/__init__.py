"""MPC controller implementations."""

from px4_mpc.controllers.standard_vtol_output import (
    govern_pusher_forward_overspeed,
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
    "govern_pusher_forward_overspeed",
    "limit_external_pusher_command",
    "limit_mc_command",
    "vertical_hover_lift",
]
