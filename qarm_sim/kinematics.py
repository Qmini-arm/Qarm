"""Forward and position-only inverse kinematics for the Qarm MuJoCo model."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass

import mujoco
import numpy as np
from numpy.typing import NDArray

from .env import JOINT_NAMES, QArmMujocoEnv

FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class PoseResult:
    position_m: FloatArray
    orientation_wxyz: FloatArray


@dataclass(frozen=True)
class IKResult:
    position_rad: FloatArray
    achieved_position_m: FloatArray
    error_m: FloatArray
    error_norm_m: float
    iterations: int
    converged: bool

    def as_dict(self) -> dict[str, object]:
        return {
            "joint_names": list(JOINT_NAMES),
            "position_rad": self.position_rad.tolist(),
            "achieved_position_m": self.achieved_position_m.tolist(),
            "error_m": self.error_m.tolist(),
            "error_norm_m": self.error_norm_m,
            "iterations": self.iterations,
            "converged": self.converged,
        }


def _validate_position(environment: QArmMujocoEnv, position_rad: Iterable[float]) -> FloatArray:
    position = np.asarray(tuple(position_rad), dtype=np.float64)
    if position.shape != (len(JOINT_NAMES),) or not np.all(np.isfinite(position)):
        raise ValueError("position_rad must contain four finite joint angles")
    ranges = environment.model.jnt_range[environment.joint_ids]
    if np.any(position < ranges[:, 0]) or np.any(position > ranges[:, 1]):
        raise ValueError("position_rad violates a MuJoCo joint range")
    return position


def _pose_from_data(environment: QArmMujocoEnv, data: mujoco.MjData) -> PoseResult:
    site_id = environment.tool_site_id
    orientation = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(orientation, data.site_xmat[site_id])
    return PoseResult(
        position_m=data.site_xpos[site_id].copy(),
        orientation_wxyz=orientation,
    )


def forward_kinematics(
    environment: QArmMujocoEnv,
    position_rad: Iterable[float],
) -> PoseResult:
    """Evaluate the MuJoCo tool site pose at a four-joint configuration."""

    position = _validate_position(environment, position_rad)
    with environment._lock:
        data = mujoco.MjData(environment.model)
        data.qpos[environment.qpos_addresses] = position
        data.qvel[environment.dof_addresses] = 0.0
        mujoco.mj_forward(environment.model, data)
        return _pose_from_data(environment, data)


def solve_position_ik(
    environment: QArmMujocoEnv,
    target_position_m: Iterable[float],
    *,
    seed_rad: Iterable[float] | None = None,
    max_iterations: int = 200,
    position_tolerance_m: float = 1e-4,
    damping: float = 1e-3,
    max_step_rad: float = 0.12,
) -> IKResult:
    """Solve a position-only IK problem with damped least-squares Jacobian steps.

    The Qarm has four joints and the solver constrains only XYZ position. The
    seed selects among redundant solutions; end-effector orientation is reported
    by FK but is intentionally not part of this solve.
    """

    target = np.asarray(tuple(target_position_m), dtype=np.float64)
    if target.shape != (3,) or not np.all(np.isfinite(target)):
        raise ValueError("target_position_m must contain three finite values")
    if isinstance(max_iterations, bool) or max_iterations <= 0:
        raise ValueError("max_iterations must be positive")
    if not np.isfinite(position_tolerance_m) or position_tolerance_m <= 0.0:
        raise ValueError("position_tolerance_m must be positive")
    if not np.isfinite(damping) or damping <= 0.0:
        raise ValueError("damping must be positive")
    if not np.isfinite(max_step_rad) or max_step_rad <= 0.0:
        raise ValueError("max_step_rad must be positive")

    if seed_rad is None:
        with environment._lock:
            seed = environment.data.qpos[environment.qpos_addresses].copy()
    else:
        seed = _validate_position(environment, seed_rad)
    ranges = environment.model.jnt_range[environment.joint_ids]
    position = seed.copy()
    site_id = environment.tool_site_id

    with environment._lock:
        data = mujoco.MjData(environment.model)
        jacobian = np.zeros((3, environment.model.nv), dtype=np.float64)
        angular_jacobian = np.zeros((3, environment.model.nv), dtype=np.float64)
        achieved = np.zeros(3, dtype=np.float64)
        error = target.copy()
        for iteration in range(1, max_iterations + 1):
            data.qpos[environment.qpos_addresses] = position
            data.qvel[environment.dof_addresses] = 0.0
            data.ctrl[:] = 0.0
            mujoco.mj_forward(environment.model, data)
            achieved[:] = data.site_xpos[site_id]
            error = target - achieved
            error_norm = float(np.linalg.norm(error))
            if error_norm <= position_tolerance_m:
                return IKResult(
                    position.copy(), achieved.copy(), error.copy(), error_norm, iteration - 1, True
                )

            mujoco.mj_jacSite(
                environment.model, data, jacobian, angular_jacobian, site_id
            )
            linear = jacobian[:, environment.dof_addresses]
            system = linear @ linear.T + (damping**2) * np.eye(3)
            step = linear.T @ np.linalg.solve(system, error)
            step_norm = float(np.linalg.norm(step))
            if step_norm > max_step_rad:
                step *= max_step_rad / step_norm

            # A short backtracking line search avoids overshooting near limits.
            previous_error_norm = error_norm
            accepted = False
            for scale in (1.0, 0.5, 0.25, 0.125):
                candidate = np.clip(position + scale * step, ranges[:, 0], ranges[:, 1])
                data.qpos[environment.qpos_addresses] = candidate
                mujoco.mj_forward(environment.model, data)
                candidate_error = target - data.site_xpos[site_id]
                if np.linalg.norm(candidate_error) < previous_error_norm:
                    position = candidate
                    accepted = True
                    break
            if not accepted:
                position = np.clip(position + 0.125 * step, ranges[:, 0], ranges[:, 1])

        data.qpos[environment.qpos_addresses] = position
        data.qvel[environment.dof_addresses] = 0.0
        mujoco.mj_forward(environment.model, data)
        achieved = data.site_xpos[site_id].copy()
        error = target - achieved
        error_norm = float(np.linalg.norm(error))
        return IKResult(
            position.copy(), achieved, error, error_norm, max_iterations, error_norm <= position_tolerance_m
        )

