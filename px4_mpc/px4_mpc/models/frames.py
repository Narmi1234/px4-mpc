"""Coordinate conversions between PX4 NED/FRD and Gazebo ENU/FLU."""

from __future__ import annotations

import numpy as np

from px4_mpc.models.standard_vtol_gz_model import quaternion_to_rotation


NED_TO_ENU = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
FRD_TO_FLU = np.diag([1.0, -1.0, -1.0])


def ned_to_enu(vector_ned: np.ndarray) -> np.ndarray:
    """Convert one vector, or an array of row vectors, from NED to ENU."""
    return np.asarray(vector_ned, dtype=float) @ NED_TO_ENU.T


def enu_to_ned(vector_enu: np.ndarray) -> np.ndarray:
    """Convert one vector, or an array of row vectors, from ENU to NED."""
    return np.asarray(vector_enu, dtype=float) @ NED_TO_ENU.T


def frd_to_flu(vector_frd: np.ndarray) -> np.ndarray:
    """Convert one vector, or an array of row vectors, from FRD to FLU."""
    return np.asarray(vector_frd, dtype=float) @ FRD_TO_FLU.T


def flu_to_frd(vector_flu: np.ndarray) -> np.ndarray:
    """Convert one vector, or an array of row vectors, from FLU to FRD."""
    return np.asarray(vector_flu, dtype=float) @ FRD_TO_FLU.T


def rotation_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    """Convert a rotation matrix to a normalized scalar-first quaternion."""
    matrix = np.asarray(rotation, dtype=float)
    trace = np.trace(matrix)
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        quaternion = np.array(
            [
                0.25 * scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            ]
        )
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = 2.0 * np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
            quaternion = np.array(
                [
                    (matrix[2, 1] - matrix[1, 2]) / scale,
                    0.25 * scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                ]
            )
        elif index == 1:
            scale = 2.0 * np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
            quaternion = np.array(
                [
                    (matrix[0, 2] - matrix[2, 0]) / scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    0.25 * scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                ]
            )
        else:
            scale = 2.0 * np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
            quaternion = np.array(
                [
                    (matrix[1, 0] - matrix[0, 1]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    0.25 * scale,
                ]
            )
    quaternion /= np.linalg.norm(quaternion)
    return quaternion if quaternion[0] >= 0.0 else -quaternion


def px4_quaternion_to_gazebo(q_ned_frd: np.ndarray) -> np.ndarray:
    """Convert PX4's FRD-to-NED quaternion to Gazebo's FLU-to-ENU one."""
    rotation_ned_frd = quaternion_to_rotation(np.asarray(q_ned_frd, dtype=float))
    rotation_enu_flu = NED_TO_ENU @ rotation_ned_frd @ FRD_TO_FLU
    return rotation_to_quaternion(rotation_enu_flu)
