"""Collision-free reachable-workspace sampling through FK."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import numpy.typing as npt

from .collision import CollisionChecker
from .model import ArmModel
from .transforms import FloatArray


@dataclass(frozen=True)
class SampledWorkspace:
    """A finite approximation of the continuous reachable position set."""

    configurations_rad: FloatArray
    positions_m: FloatArray
    requested_samples: int

    @property
    def accepted_samples(self) -> int:
        return len(self.positions_m)

    @property
    def collision_free_fraction(self) -> float:
        return self.accepted_samples / max(1, self.requested_samples)

    @property
    def bounds(self) -> tuple[FloatArray, FloatArray]:
        if not len(self.positions_m):
            raise ValueError("workspace contains no collision-free samples")
        return self.positions_m.min(axis=0), self.positions_m.max(axis=0)

    def nearest_distance(self, point: npt.ArrayLike) -> float:
        target = np.asarray(point, dtype=np.float64).reshape(3)
        return float(np.min(np.linalg.norm(self.positions_m - target, axis=1)))

    def save(self, path: str | Path) -> Path:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            destination,
            configurations_rad=self.configurations_rad,
            positions_m=self.positions_m,
            requested_samples=np.array(self.requested_samples),
        )
        return destination


def sample_workspace(
    model: ArmModel,
    collision: CollisionChecker,
    *,
    count: int = 20000,
    seed: int = 0,
) -> SampledWorkspace:
    """Uniformly sample soft-limit joint space and retain collision-free FK poses."""
    if count <= 0:
        raise ValueError("workspace sample count must be positive")
    rng = np.random.default_rng(seed)
    configurations: list[FloatArray] = []
    positions: list[FloatArray] = []
    for _ in range(count):
        q = model.random_configuration(rng)
        if collision.is_free(q):
            configurations.append(q)
            positions.append(model.fk(q)[:3, 3])
    return SampledWorkspace(
        np.asarray(configurations, dtype=np.float64).reshape(-1, model.dof),
        np.asarray(positions, dtype=np.float64).reshape(-1, 3),
        count,
    )
