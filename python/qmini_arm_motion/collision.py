"""Conservative self-collision checking for URDF collision primitives."""

from __future__ import annotations

import itertools
from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

from .model import ArmModel
from .transforms import FloatArray


@dataclass(frozen=True)
class CollisionPair:
    link_a: str
    link_b: str
    penetration_m: float

    def __str__(self) -> str:
        return f"{self.link_a}<->{self.link_b} ({self.penetration_m * 1000.0:.1f} mm)"


def _obb_penetration(
    pose_a: FloatArray,
    half_a: FloatArray,
    pose_b: FloatArray,
    half_b: FloatArray,
) -> float:
    """Separating-axis test for two oriented boxes."""
    rotation_a, rotation_b = pose_a[:3, :3], pose_b[:3, :3]
    delta = pose_b[:3, 3] - pose_a[:3, 3]
    axes = [rotation_a[:, i] for i in range(3)]
    axes.extend(rotation_b[:, i] for i in range(3))
    for i in range(3):
        for j in range(3):
            axis = np.cross(rotation_a[:, i], rotation_b[:, j])
            norm = float(np.linalg.norm(axis))
            if norm > 1e-10:
                axes.append(axis / norm)

    minimum = np.inf
    for axis in axes:
        reach_a = float(np.abs(rotation_a.T @ axis) @ half_a)
        reach_b = float(np.abs(rotation_b.T @ axis) @ half_b)
        overlap = reach_a + reach_b - abs(float(delta @ axis))
        if overlap <= 0.0:
            return 0.0
        minimum = min(minimum, overlap)
    return float(minimum)


class CollisionChecker:
    """Check self-collision and continuous joint-space edges.

    URDF boxes are tested exactly. Cylinders are replaced with their enclosing
    oriented boxes, which is deliberately conservative: it may reject a narrow
    valid passage, but it will not miss a collision because a motor housing was
    approximated too small. Direct parent-child pairs are ignored because they
    touch by construction; every non-adjacent pair remains active.
    """

    def __init__(
        self,
        model: ArmModel,
        *,
        margin_m: float = 0.002,
        edge_resolution_rad: float = np.radians(2.0),
    ) -> None:
        if margin_m < 0.0 or edge_resolution_rad <= 0.0:
            raise ValueError("collision margin must be non-negative and resolution positive")
        self.model = model
        self.margin_m = float(margin_m)
        self.edge_resolution_rad = float(edge_resolution_rad)
        self._geometries = {
            name: tuple(
                (geometry.origin, geometry.size / 2.0 + margin_m) for geometry in link.collisions
            )
            for name, link in model.links.items()
            if link.collisions
        }
        adjacent = {frozenset((joint.parent, joint.child)) for joint in model.chain_joints}
        self._pairs = tuple(
            (left, right)
            for left, right in itertools.combinations(sorted(self._geometries), 2)
            if frozenset((left, right)) not in adjacent
        )

    def check(self, q: npt.ArrayLike) -> tuple[CollisionPair, ...]:
        poses = self.model.link_poses(q)
        world = {
            name: tuple(poses[name] @ local for local, _half in geometries)
            for name, geometries in self._geometries.items()
        }
        found: list[CollisionPair] = []
        for link_a, link_b in self._pairs:
            deepest = 0.0
            for index_a, (_local_a, half_a) in enumerate(self._geometries[link_a]):
                pose_a = world[link_a][index_a]
                radius_a = float(np.linalg.norm(half_a))
                for index_b, (_local_b, half_b) in enumerate(self._geometries[link_b]):
                    pose_b = world[link_b][index_b]
                    if np.linalg.norm(pose_a[:3, 3] - pose_b[:3, 3]) > (
                        radius_a + np.linalg.norm(half_b)
                    ):
                        continue
                    deepest = max(deepest, _obb_penetration(pose_a, half_a, pose_b, half_b))
            if deepest > 0.0:
                found.append(CollisionPair(link_a, link_b, deepest))
        return tuple(found)

    def is_free(self, q: npt.ArrayLike) -> bool:
        return not self.check(q)

    def segment_is_free(self, start: npt.ArrayLike, goal: npt.ArrayLike) -> bool:
        """Discretely validate an entire joint-space edge, including both ends."""
        q0 = np.asarray(start, dtype=np.float64).reshape(self.model.dof)
        q1 = np.asarray(goal, dtype=np.float64).reshape(self.model.dof)
        samples = max(1, int(np.ceil(np.max(np.abs(q1 - q0)) / self.edge_resolution_rad)))
        return all(self.is_free(q0 + (q1 - q0) * (index / samples)) for index in range(samples + 1))

    def path_is_free(self, path: npt.ArrayLike) -> bool:
        values = np.asarray(path, dtype=np.float64).reshape(-1, self.model.dof)
        return all(
            self.segment_is_free(values[index], values[index + 1])
            for index in range(len(values) - 1)
        )
