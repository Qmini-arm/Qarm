"""Compare the hand-tuned gravity formula with MuJoCo inverse dynamics."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from .env import JOINT_NAMES, QArmMujocoEnv

FloatArray = NDArray[np.float64]
DEFAULT_PI_COEFFICIENTS = (4.4, 1.712549, 0.0)


def empirical_gravity_compensation(
    position_rad: Iterable[float],
    pi_coefficients: Iterable[float] = DEFAULT_PI_COEFFICIENTS,
) -> FloatArray:
    """Evaluate ``functions.py``'s PI formula in joint_1..joint_4 order.

    The legacy formula only produces torques for motor IDs 1..3.  Its omitted
    motor ID 0 is represented as zero here so the result can be compared with
    MuJoCo's four generalized joint torques without silently shifting axes.
    """

    position = np.asarray(tuple(position_rad), dtype=np.float64)
    coefficients = np.asarray(tuple(pi_coefficients), dtype=np.float64)
    if position.shape != (4,) or not np.all(np.isfinite(position)):
        raise ValueError("position_rad must contain four finite values")
    if coefficients.shape != (3,) or not np.all(np.isfinite(coefficients)):
        raise ValueError("pi_coefficients must contain three finite values")

    pi_1, pi_2, pi_3 = coefficients
    q_joint_2, q_joint_3, q_joint_4 = position[1:]
    angle_link_2 = q_joint_2
    angle_link_3 = q_joint_2 - q_joint_3
    angle_link_6 = q_joint_2 - q_joint_3 + q_joint_4
    tau_joint_4 = pi_3 * np.sin(angle_link_6)
    tau_joint_3 = pi_2 * np.sin(angle_link_3) + tau_joint_4
    tau_joint_2 = -(pi_1 * np.sin(angle_link_2) + tau_joint_3)
    return np.asarray(
        [0.0, tau_joint_2, tau_joint_3, tau_joint_4], dtype=np.float64
    )


@dataclass(frozen=True)
class GravityComparison:
    position_rad: FloatArray
    mujoco_torque_nm: FloatArray
    empirical_torque_nm: FloatArray
    error_nm: FloatArray
    absolute_error_nm: FloatArray
    max_absolute_error_nm: float
    rmse_nm: float

    def as_dict(self) -> dict[str, object]:
        return {
            "joint_names": list(JOINT_NAMES),
            "position_rad": self.position_rad.tolist(),
            "mujoco_inverse_dynamics_torque_nm": self.mujoco_torque_nm.tolist(),
            "empirical_torque_nm": self.empirical_torque_nm.tolist(),
            "error_empirical_minus_mujoco_nm": self.error_nm.tolist(),
            "absolute_error_nm": self.absolute_error_nm.tolist(),
            "max_absolute_error_nm": self.max_absolute_error_nm,
            "rmse_nm": self.rmse_nm,
        }


def compare_gravity_compensation(
    environment: QArmMujocoEnv,
    position_rad: Iterable[float],
    pi_coefficients: Iterable[float] = DEFAULT_PI_COEFFICIENTS,
) -> GravityComparison:
    """Compare both methods at one fixed joint configuration."""

    position = np.asarray(tuple(position_rad), dtype=np.float64)
    mujoco_torque = environment.inverse_dynamics_gravity(position)
    empirical_torque = empirical_gravity_compensation(position, pi_coefficients)
    error = empirical_torque - mujoco_torque
    absolute_error = np.abs(error)
    return GravityComparison(
        position_rad=position.copy(),
        mujoco_torque_nm=mujoco_torque,
        empirical_torque_nm=empirical_torque,
        error_nm=error,
        absolute_error_nm=absolute_error,
        max_absolute_error_nm=float(np.max(absolute_error)),
        rmse_nm=float(np.sqrt(np.mean(error**2))),
    )

