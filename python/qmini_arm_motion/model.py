"""URDF-backed serial-chain model with ``base_link`` fixed at identity.

This follows the proven FK/Jacobian layout in ``/home/wyt06/Qmini-arm/arm_ik``
but supports this repository's box and cylinder collision geometry and uses the
URDF soft limits as planning limits.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import numpy.typing as npt

from .transforms import FloatArray, axis_angle_to_matrix, make_transform


def _numbers(text: str | None, count: int, default: tuple[float, ...]) -> FloatArray:
    values = default if text is None else tuple(float(value) for value in text.split())
    if len(values) != count:
        raise ValueError(f"expected {count} values, got {values}")
    return np.asarray(values, dtype=np.float64)


def _origin(element: ET.Element | None) -> FloatArray:
    if element is None or (node := element.find("origin")) is None:
        return np.eye(4, dtype=np.float64)
    return make_transform(
        _numbers(node.get("xyz"), 3, (0.0, 0.0, 0.0)),
        _numbers(node.get("rpy"), 3, (0.0, 0.0, 0.0)),
    )


@dataclass(frozen=True)
class CollisionGeometry:
    kind: str
    origin: FloatArray
    size: FloatArray


@dataclass(frozen=True)
class Link:
    name: str
    collisions: tuple[CollisionGeometry, ...]


@dataclass(frozen=True)
class Joint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin: FloatArray
    axis: FloatArray
    lower: float
    upper: float
    velocity: float

    @property
    def actuated(self) -> bool:
        return self.joint_type in {"revolute", "continuous", "prismatic"}


@dataclass(frozen=True)
class ChainState:
    link_poses: dict[str, FloatArray]
    axis_origins: FloatArray
    axis_directions: FloatArray
    tip_pose: FloatArray


class ArmModel:
    """A six-axis URDF chain expressed entirely in the ``base_link`` frame."""

    def __init__(
        self,
        urdf_path: str | Path,
        *,
        base_link: str = "base_link",
        tip_link: str = "tool0",
        use_soft_limits: bool = True,
    ) -> None:
        self.urdf_path = Path(urdf_path).resolve()
        self.base_link = base_link
        self.tip_link = tip_link
        self.links, all_joints = self._parse(self.urdf_path, use_soft_limits)
        if base_link not in self.links or tip_link not in self.links:
            raise ValueError(f"URDF must contain {base_link!r} and {tip_link!r}")

        by_child = {joint.child: joint for joint in all_joints}
        reverse_chain: list[Joint] = []
        current = tip_link
        while current != base_link:
            if current not in by_child:
                raise ValueError(f"{tip_link!r} is not connected to {base_link!r}")
            joint = by_child[current]
            reverse_chain.append(joint)
            current = joint.parent
        self.chain_joints = tuple(reversed(reverse_chain))
        self.joints = tuple(joint for joint in self.chain_joints if joint.actuated)
        if len(self.joints) != 6:
            raise ValueError(f"expected a 6-DoF chain, found {len(self.joints)}")
        self.joint_names = tuple(joint.name for joint in self.joints)
        self.lower = np.array([joint.lower for joint in self.joints])
        self.upper = np.array([joint.upper for joint in self.joints])
        self.velocity = np.array([joint.velocity for joint in self.joints])
        self._actuated_index = {name: index for index, name in enumerate(self.joint_names)}

        children: dict[str, list[Joint]] = {}
        for joint in all_joints:
            children.setdefault(joint.parent, []).append(joint)
        self._children = children
        self._validate_tree(all_joints)

    @property
    def dof(self) -> int:
        return len(self.joints)

    @property
    def mid_range(self) -> FloatArray:
        return 0.5 * (self.lower + self.upper)

    def clamp(self, q: npt.ArrayLike) -> FloatArray:
        return np.clip(np.asarray(q, dtype=np.float64).reshape(self.dof), self.lower, self.upper)

    def within_limits(self, q: npt.ArrayLike, tolerance: float = 1e-9) -> bool:
        values = np.asarray(q, dtype=np.float64).reshape(-1, self.dof)
        return bool(
            np.all(values >= self.lower[None, :] - tolerance)
            and np.all(values <= self.upper[None, :] + tolerance)
        )

    def random_configuration(self, rng: np.random.Generator) -> FloatArray:
        return rng.uniform(self.lower, self.upper)

    def chain_state(self, q: npt.ArrayLike) -> ChainState:
        values = np.asarray(q, dtype=np.float64).reshape(self.dof)
        if not np.all(np.isfinite(values)):
            raise ValueError("joint configuration contains a non-finite value")
        link_poses: dict[str, FloatArray] = {self.base_link: np.eye(4)}
        origins = np.zeros((self.dof, 3), dtype=np.float64)
        directions = np.zeros((self.dof, 3), dtype=np.float64)
        stack = [self.base_link]
        while stack:
            parent = stack.pop()
            parent_pose = link_poses[parent]
            for joint in self._children.get(parent, ()):
                joint_pose = parent_pose @ joint.origin
                local_motion = np.eye(4)
                index = self._actuated_index.get(joint.name)
                if index is not None:
                    if joint.joint_type not in {"revolute", "continuous"}:
                        raise NotImplementedError("only revolute joints are supported")
                    local_motion[:3, :3] = axis_angle_to_matrix(joint.axis, values[index])
                    origins[index] = joint_pose[:3, 3]
                    directions[index] = joint_pose[:3, :3] @ joint.axis
                link_poses[joint.child] = joint_pose @ local_motion
                stack.append(joint.child)
        return ChainState(link_poses, origins, directions, link_poses[self.tip_link])

    def fk(self, q: npt.ArrayLike) -> FloatArray:
        """Return the ``base_link -> tool0`` homogeneous transform."""
        return self.chain_state(q).tip_pose

    def link_poses(self, q: npt.ArrayLike) -> dict[str, FloatArray]:
        return self.chain_state(q).link_poses

    def jacobian(self, q: npt.ArrayLike) -> FloatArray:
        state = self.chain_state(q)
        delta = state.tip_pose[:3, 3] - state.axis_origins
        jacobian = np.empty((6, self.dof), dtype=np.float64)
        jacobian[:3] = np.cross(state.axis_directions, delta).T
        jacobian[3:] = state.axis_directions.T
        return jacobian

    def _validate_tree(self, joints: tuple[Joint, ...]) -> None:
        parent_of: dict[str, str] = {}
        for joint in joints:
            if joint.child in parent_of:
                raise ValueError(f"link {joint.child!r} has more than one parent")
            parent_of[joint.child] = joint.parent
            if joint.parent not in self.links or joint.child not in self.links:
                raise ValueError(f"joint {joint.name!r} references a missing link")
        seen: set[str] = set()
        stack = [self.base_link]
        while stack:
            link = stack.pop()
            if link in seen:
                raise ValueError(f"cycle detected at link {link!r}")
            seen.add(link)
            stack.extend(joint.child for joint in self._children.get(link, ()))
        if self.tip_link not in seen:
            raise ValueError(f"tip {self.tip_link!r} is unreachable from base")

    @staticmethod
    def _parse(path: Path, use_soft_limits: bool) -> tuple[dict[str, Link], tuple[Joint, ...]]:
        root = ET.parse(path).getroot()
        links: dict[str, Link] = {}
        for node in root.findall("link"):
            name = node.get("name")
            if not name or name in links:
                raise ValueError(f"invalid or duplicate link name: {name!r}")
            geometries: list[CollisionGeometry] = []
            for collision in node.findall("collision"):
                geometry = collision.find("geometry")
                if geometry is None:
                    continue
                if (box := geometry.find("box")) is not None:
                    size = _numbers(box.get("size"), 3, (0.0, 0.0, 0.0))
                    kind = "box"
                elif (cylinder := geometry.find("cylinder")) is not None:
                    radius = float(cylinder.get("radius", "0"))
                    length = float(cylinder.get("length", "0"))
                    size = np.array([2.0 * radius, 2.0 * radius, length])
                    kind = "cylinder"
                else:
                    continue
                if np.any(size <= 0.0):
                    raise ValueError(f"link {name!r} has invalid collision dimensions")
                geometries.append(CollisionGeometry(kind, _origin(collision), size))
            links[name] = Link(name, tuple(geometries))

        joints: list[Joint] = []
        for node in root.findall("joint"):
            name, kind = node.get("name"), node.get("type")
            parent_node, child_node = node.find("parent"), node.find("child")
            if not name or not kind or parent_node is None or child_node is None:
                raise ValueError("joint is missing name/type/parent/child")
            lower, upper, velocity = -np.pi, np.pi, 0.0
            if (limit := node.find("limit")) is not None:
                lower = float(limit.get("lower", -np.pi))
                upper = float(limit.get("upper", np.pi))
                velocity = float(limit.get("velocity", "0"))
            if use_soft_limits and (safety := node.find("safety_controller")) is not None:
                lower = max(lower, float(safety.get("soft_lower_limit", lower)))
                upper = min(upper, float(safety.get("soft_upper_limit", upper)))
            if lower >= upper and kind != "fixed":
                raise ValueError(f"joint {name!r} has an empty limit interval")
            axis_node = node.find("axis")
            axis = _numbers(
                None if axis_node is None else axis_node.get("xyz"),
                3,
                (1.0, 0.0, 0.0),
            )
            axis_norm = float(np.linalg.norm(axis))
            if kind != "fixed" and axis_norm <= 1e-12:
                raise ValueError(f"joint {name!r} has a zero axis")
            if axis_norm > 0.0:
                axis /= axis_norm
            joints.append(
                Joint(
                    name,
                    kind,
                    str(parent_node.get("link")),
                    str(child_node.get("link")),
                    _origin(node),
                    axis,
                    lower,
                    upper,
                    velocity,
                )
            )
        return links, tuple(joints)
