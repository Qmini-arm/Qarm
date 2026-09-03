"""Small SE(3) helpers using the URDF fixed-axis RPY convention."""

from __future__ import annotations

import numpy as np
import numpy.typing as npt

FloatArray = npt.NDArray[np.float64]


def rpy_to_matrix(rpy: npt.ArrayLike) -> FloatArray:
    """Return ``Rz(yaw) @ Ry(pitch) @ Rx(roll)``."""
    roll, pitch, yaw = np.asarray(rpy, dtype=np.float64).reshape(3)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def make_transform(
    xyz: npt.ArrayLike = (0.0, 0.0, 0.0),
    rpy: npt.ArrayLike = (0.0, 0.0, 0.0),
) -> FloatArray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rpy_to_matrix(rpy)
    transform[:3, 3] = np.asarray(xyz, dtype=np.float64).reshape(3)
    return transform


def axis_angle_to_matrix(axis: npt.ArrayLike, angle: float) -> FloatArray:
    unit = np.asarray(axis, dtype=np.float64).reshape(3)
    norm = float(np.linalg.norm(unit))
    if norm <= 1e-12:
        raise ValueError("joint axis must be non-zero")
    unit /= norm
    x, y, z = unit
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)


def rotation_to_rpy(rotation: npt.ArrayLike) -> FloatArray:
    rot = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    pitch = np.arcsin(np.clip(-rot[2, 0], -1.0, 1.0))
    if abs(np.cos(pitch)) < 1e-9:
        return np.array([0.0, pitch, np.arctan2(-rot[0, 1], rot[1, 1])])
    return np.array([np.arctan2(rot[2, 1], rot[2, 2]), pitch, np.arctan2(rot[1, 0], rot[0, 0])])
