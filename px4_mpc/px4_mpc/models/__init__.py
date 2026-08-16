"""Dynamic models used by px4_mpc."""

from px4_mpc.models.mc_forward_profile import McForwardProfile
from px4_mpc.models.mc_forward_profile import McForwardSample
from px4_mpc.models.mc_forward_profile import mc_forward_reference_state
from px4_mpc.models.standard_vtol_gz_model import StandardVtolGazeboModel
from px4_mpc.models.standard_vtol_gz_model import StandardVtolTransitionRateModel
from px4_mpc.models.standard_vtol_trim import StandardVtolTrimSolver
from px4_mpc.models.standard_vtol_trim import TransitionTrimPoint
try:
    from px4_mpc.models.standard_vtol_casadi_model import (
        StandardVtolTransitionCasadiModel,
    )
except ImportError:  # CasADi is optional for plant validation utilities.
    StandardVtolTransitionCasadiModel = None

__all__ = [
    "McForwardProfile",
    "McForwardSample",
    "mc_forward_reference_state",
    "StandardVtolGazeboModel",
    "StandardVtolTransitionRateModel",
    "StandardVtolTrimSolver",
    "TransitionTrimPoint",
    "StandardVtolTransitionCasadiModel",
]
