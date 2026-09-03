"""Collision-filtered position IK using adaptive damped least squares."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np
import numpy.typing as npt

from .collision import CollisionChecker
from .model import ArmModel
from .transforms import FloatArray


class IKStatus(Enum):
    CONVERGED = "converged"
    COLLISION = "collision"
    LIMIT_BLOCKED = "limit_blocked"
    MAX_ITERATIONS = "max_iterations"

    @property
    def usable(self) -> bool:
        return self is IKStatus.CONVERGED


@dataclass(frozen=True)
class IKConfig:
    tolerance_m: float = 0.001
    max_iterations: int = 250
    restarts: int = 40
    damping_initial: float = 0.02
    damping_min: float = 1e-4
    damping_max: float = 100.0
    max_step_rad: float = 0.18
    joint_centering: float = 0.03
    random_seed: int = 0

    def __post_init__(self) -> None:
        if self.tolerance_m <= 0.0 or self.max_iterations < 1 or self.restarts < 1:
            raise ValueError("IK tolerance must be positive and iteration counts non-zero")
        if not 0.0 < self.damping_min <= self.damping_initial <= self.damping_max:
            raise ValueError("IK damping must satisfy 0 < min <= initial <= max")
        if self.max_step_rad <= 0.0 or self.joint_centering < 0.0:
            raise ValueError("IK step must be positive and joint centering non-negative")


@dataclass(frozen=True)
class IKResult:
    status: IKStatus
    q: FloatArray
    position_error_m: float
    iterations: int
    attempts: int
    collisions: tuple[str, ...] = ()

    @property
    def success(self) -> bool:
        return self.status.usable


class PositionIKSolver:
    """Solve a target point while leaving end-effector orientation unconstrained."""

    def __init__(
        self,
        model: ArmModel,
        collision: CollisionChecker,
        config: IKConfig | None = None,
    ) -> None:
        self.model = model
        self.collision = collision
        self.config = config or IKConfig()

    def solve(self, target_position: npt.ArrayLike, seed: npt.ArrayLike) -> IKResult:
        target = np.asarray(target_position, dtype=np.float64).reshape(3)
        start = self.model.clamp(seed)
        if not np.all(np.isfinite(target)) or not np.all(np.isfinite(start)):
            raise ValueError("IK target and seed must be finite")
        rng = np.random.default_rng(self.config.random_seed)
        seeds = [start, self.model.mid_range]
        seeds.extend(
            self.model.random_configuration(rng)
            for _ in range(max(0, self.config.restarts - len(seeds)))
        )

        best: IKResult | None = None
        for attempt, candidate_seed in enumerate(seeds, start=1):
            q, iterations = self._solve_once(target, candidate_seed, start)
            error = float(np.linalg.norm(target - self.model.fk(q)[:3, 3]))
            collisions = tuple(str(pair) for pair in self.collision.check(q))
            if error <= self.config.tolerance_m and not collisions:
                return IKResult(IKStatus.CONVERGED, q, error, iterations, attempt)
            if collisions and error <= self.config.tolerance_m:
                status = IKStatus.COLLISION
            elif np.any(np.isclose(q, self.model.lower, atol=1e-6)) or np.any(
                np.isclose(q, self.model.upper, atol=1e-6)
            ):
                status = IKStatus.LIMIT_BLOCKED
            else:
                status = IKStatus.MAX_ITERATIONS
            result = IKResult(status, q, error, iterations, attempt, collisions)
            if best is None or self._rank(result, start) < self._rank(best, start):
                best = result
        assert best is not None
        return best

    def _solve_once(
        self, target: FloatArray, seed: FloatArray, preferred: FloatArray
    ) -> tuple[FloatArray, int]:
        cfg = self.config
        q = self.model.clamp(seed)
        damping = cfg.damping_initial
        error = target - self.model.fk(q)[:3, 3]
        residual = float(np.linalg.norm(error))

        for iteration in range(1, cfg.max_iterations + 1):
            if residual <= cfg.tolerance_m:
                return q, iteration - 1
            jacobian = self.model.jacobian(q)[:3]
            regularized = jacobian @ jacobian.T + damping * damping * np.eye(3)
            try:
                solved = np.linalg.solve(regularized, error)
                pseudo_inverse = jacobian.T @ np.linalg.solve(regularized, np.eye(3))
            except np.linalg.LinAlgError:
                damping = min(cfg.damping_max, damping * 3.0)
                continue
            primary = jacobian.T @ solved
            nullspace = np.eye(self.model.dof) - pseudo_inverse @ jacobian
            secondary = cfg.joint_centering * (nullspace @ (preferred - q))
            step = primary + secondary
            norm = float(np.linalg.norm(step))
            if norm > cfg.max_step_rad:
                step *= cfg.max_step_rad / norm
            trial = self.model.clamp(q + step)
            trial_error = target - self.model.fk(trial)[:3, 3]
            trial_residual = float(np.linalg.norm(trial_error))
            if trial_residual < residual:
                q, error, residual = trial, trial_error, trial_residual
                damping = max(cfg.damping_min, damping * 0.5)
            else:
                damping = min(cfg.damping_max, damping * 2.5)
                if damping >= cfg.damping_max:
                    return q, iteration
        return q, cfg.max_iterations

    @staticmethod
    def _rank(result: IKResult, preferred: FloatArray) -> tuple[float, float, float]:
        collision_penalty = 1.0 if result.collisions else 0.0
        return (
            collision_penalty,
            result.position_error_m,
            float(np.linalg.norm(result.q - preferred)),
        )
