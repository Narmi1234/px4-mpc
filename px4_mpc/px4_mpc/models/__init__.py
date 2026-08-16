"""Dynamic models used by px4_mpc."""

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
    "StandardVtolGazeboModel",
    "StandardVtolTransitionRateModel",
    "StandardVtolTrimSolver",
    "TransitionTrimPoint",
    "StandardVtolTransitionCasadiModel",
]
