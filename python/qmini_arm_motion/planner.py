"""IK-driven Cartesian planning with a collision-aware joint-space fallback."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

from .collision import CollisionChecker
from .ik import IKResult, PositionIKSolver
from .model import ArmModel
from .transforms import FloatArray


@dataclass(frozen=True)
class PlannerConfig:
    cartesian_step_m: float = 0.025
    rrt_step_rad: float = 0.20
    rrt_iterations: int = 5000
    shortcut_attempts: int = 160
    velocity_limit_rad_s: float = 0.5
    acceleration_limit_rad_s2: float = 1.0
    control_period_s: float = 0.02
    random_seed: int = 0

    def __post_init__(self) -> None:
        positive = (
            self.cartesian_step_m,
            self.rrt_step_rad,
            self.velocity_limit_rad_s,
            self.acceleration_limit_rad_s2,
            self.control_period_s,
        )
        if any(value <= 0.0 for value in positive):
            raise ValueError("planner distances, limits and period must be positive")
        if self.rrt_iterations < 1 or self.shortcut_attempts < 0:
            raise ValueError("RRT iterations must be positive and shortcuts non-negative")


@dataclass(frozen=True)
class TimedTrajectory:
    times_s: FloatArray
    positions_rad: FloatArray
    velocities_rad_s: FloatArray

    @property
    def duration_s(self) -> float:
        return float(self.times_s[-1])


@dataclass(frozen=True)
class MotionPlan:
    target_position_m: FloatArray
    ik: IKResult
    path_kind: str
    waypoints_rad: FloatArray
    trajectory: TimedTrajectory


@dataclass
class _Tree:
    nodes: list[FloatArray]
    parents: list[int]
    rooted_at_start: bool

    def path_to(self, index: int) -> list[FloatArray]:
        path: list[FloatArray] = []
        while index >= 0:
            path.append(self.nodes[index])
            index = self.parents[index]
        return list(reversed(path))


class MotionPlanner:
    """Plan from a current joint state to a Cartesian target point.

    The module first solves closely spaced Cartesian waypoints with each IK
    seeded from the previous solution. If that straight end-effector path is
    unreachable or self-colliding, it solves a collision-free goal IK and uses
    bidirectional RRT-Connect in joint space. Callers get the same result type
    in either case.
    """

    def __init__(
        self,
        model: ArmModel,
        collision: CollisionChecker,
        ik: PositionIKSolver | None = None,
        config: PlannerConfig | None = None,
    ) -> None:
        self.model = model
        self.collision = collision
        self.ik = ik or PositionIKSolver(model, collision)
        self.config = config or PlannerConfig()

    def plan(self, start_q: npt.ArrayLike, target_position_m: npt.ArrayLike) -> MotionPlan:
        start = np.asarray(start_q, dtype=np.float64).reshape(self.model.dof)
        target = np.asarray(target_position_m, dtype=np.float64).reshape(3)
        if not np.all(np.isfinite(start)) or not np.all(np.isfinite(target)):
            raise ValueError("start configuration and target position must be finite")
        if not self.model.within_limits(start):
            raise ValueError("start configuration violates the URDF soft limits")
        collisions = self.collision.check(start)
        if collisions:
            raise ValueError(
                "start configuration self-collides: " + ", ".join(map(str, collisions))
            )

        cartesian = self._cartesian_ik_path(start, target)
        if cartesian is not None:
            waypoints, result = cartesian
            kind = "cartesian_ik"
        else:
            result = self.ik.solve(target, start)
            if not result.success:
                detail = (
                    ", ".join(result.collisions) if result.collisions else "no free IK solution"
                )
                raise ValueError(
                    f"target IK failed: {result.status.value}, "
                    f"error={result.position_error_m * 1000.0:.2f} mm; {detail}"
                )
            waypoints = self._joint_path(start, result.q)
            kind = "joint_rrt" if len(waypoints) > 2 else "joint_direct"

        trajectory = quintic_time_parameterize(
            waypoints,
            np.minimum(self.model.velocity, self.config.velocity_limit_rad_s),
            self.config.acceleration_limit_rad_s2,
            self.config.control_period_s,
        )
        if not self.collision.path_is_free(trajectory.positions_rad):
            raise RuntimeError("internal error: time-parameterized path is not collision-free")
        return MotionPlan(target, result, kind, waypoints, trajectory)

    def _cartesian_ik_path(
        self, start: FloatArray, target: FloatArray
    ) -> tuple[FloatArray, IKResult] | None:
        origin = self.model.fk(start)[:3, 3]
        distance = float(np.linalg.norm(target - origin))
        segments = max(1, int(np.ceil(distance / self.config.cartesian_step_m)))
        path = [start]
        last_result: IKResult | None = None
        for index in range(1, segments + 1):
            point = origin + (target - origin) * (index / segments)
            last_result = self.ik.solve(point, path[-1])
            if not last_result.success or not self.collision.segment_is_free(
                path[-1], last_result.q
            ):
                return None
            path.append(last_result.q)
        assert last_result is not None
        return np.asarray(path), last_result

    def _joint_path(self, start: FloatArray, goal: FloatArray) -> FloatArray:
        if self.collision.segment_is_free(start, goal):
            return np.asarray([start, goal])
        path = self._rrt_connect(start, goal)
        if path is None:
            raise ValueError("no self-collision-free joint path found within the RRT budget")
        return self._shortcut(path)

    def _rrt_connect(self, start: FloatArray, goal: FloatArray) -> FloatArray | None:
        rng = np.random.default_rng(self.config.random_seed)
        first = _Tree([start.copy()], [-1], True)
        second = _Tree([goal.copy()], [-1], False)
        for _ in range(self.config.rrt_iterations):
            sample = self.model.random_configuration(rng)
            index_a = self._extend(first, sample)
            if index_a is not None:
                index_b = self._connect(second, first.nodes[index_a])
                if index_b is not None:
                    path_a, path_b = first.path_to(index_a), second.path_to(index_b)
                    if first.rooted_at_start:
                        return np.asarray(path_a + list(reversed(path_b[:-1])))
                    return np.asarray(path_b + list(reversed(path_a[:-1])))
            first, second = second, first
        return None

    def _nearest(self, tree: _Tree, target: FloatArray) -> int:
        nodes = np.asarray(tree.nodes)
        scale = np.maximum(self.model.upper - self.model.lower, 1e-12)
        return int(np.argmin(np.linalg.norm((nodes - target) / scale, axis=1)))

    def _extend(self, tree: _Tree, target: FloatArray) -> int | None:
        nearest = self._nearest(tree, target)
        source = tree.nodes[nearest]
        delta = target - source
        distance = float(np.linalg.norm(delta))
        candidate = (
            target.copy()
            if distance <= self.config.rrt_step_rad
            else (source + delta * (self.config.rrt_step_rad / distance))
        )
        if not self.collision.segment_is_free(source, candidate):
            return None
        tree.nodes.append(candidate)
        tree.parents.append(nearest)
        return len(tree.nodes) - 1

    def _connect(self, tree: _Tree, target: FloatArray) -> int | None:
        while True:
            index = self._extend(tree, target)
            if index is None:
                return None
            if np.linalg.norm(tree.nodes[index] - target) <= 1e-10:
                return index

    def _shortcut(self, path: FloatArray) -> FloatArray:
        points = [row.copy() for row in path]
        rng = np.random.default_rng(self.config.random_seed + 1)
        for _ in range(self.config.shortcut_attempts):
            if len(points) <= 2:
                break
            left, right = sorted(rng.choice(len(points), size=2, replace=False))
            if right <= left + 1:
                continue
            if self.collision.segment_is_free(points[left], points[right]):
                points[left + 1 : right] = []
        return np.asarray(points)


def quintic_time_parameterize(
    waypoints: npt.ArrayLike,
    velocity_limits_rad_s: npt.ArrayLike,
    acceleration_limit_rad_s2: float,
    control_period_s: float,
) -> TimedTrajectory:
    """Stop-smoothly-at-waypoints trajectory with bounded quintic derivatives."""
    path = np.asarray(waypoints, dtype=np.float64)
    if path.ndim != 2 or len(path) < 2:
        raise ValueError("at least two joint waypoints are required")
    limits = np.asarray(velocity_limits_rad_s, dtype=np.float64).reshape(path.shape[1])
    if not np.all(np.isfinite(path)) or not np.all(np.isfinite(limits)):
        raise ValueError("trajectory inputs must be finite")
    if np.any(limits <= 0.0) or acceleration_limit_rad_s2 <= 0.0 or control_period_s <= 0.0:
        raise ValueError("trajectory limits and control period must be positive")

    times = [0.0]
    positions = [path[0].copy()]
    velocities = [np.zeros(path.shape[1])]
    elapsed = 0.0
    max_quintic_speed = 1.875
    max_quintic_acceleration = 10.0 / np.sqrt(3.0)
    for start, goal in zip(path[:-1], path[1:], strict=True):
        delta = goal - start
        if np.max(np.abs(delta)) <= 1e-12:
            continue
        duration_velocity = float(np.max(max_quintic_speed * np.abs(delta) / limits))
        duration_acceleration = float(
            np.sqrt(np.max(max_quintic_acceleration * np.abs(delta) / acceleration_limit_rad_s2))
        )
        duration = max(control_period_s, duration_velocity, duration_acceleration)
        sample_count = max(1, int(np.ceil(duration / control_period_s)))
        duration = sample_count * control_period_s
        for sample in range(1, sample_count + 1):
            local_time = duration * sample / sample_count
            u = local_time / duration
            s = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
            ds_dt = (30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4) / duration
            times.append(elapsed + local_time)
            positions.append(start + delta * s)
            velocities.append(delta * ds_dt)
        elapsed += duration
    if len(times) == 1:
        times.append(control_period_s)
        positions.append(path[-1].copy())
        velocities.append(np.zeros(path.shape[1]))
    velocities[-1] = np.zeros(path.shape[1])
    return TimedTrajectory(np.asarray(times), np.asarray(positions), np.asarray(velocities))
