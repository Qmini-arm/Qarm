"""Offline motion-planning facade shared by the MuJoCo command line tools.

This module deliberately has no telemetry or motor-bus imports.  It consumes a
URDF and produces kinematic results or joint trajectories; executing those
trajectories on hardware remains the responsibility of the guarded C++ tools.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

from qmini_arm_motion import ArmModel, CollisionChecker, MotionPlanner, PlannerConfig
from qmini_arm_motion.planner import TimedTrajectory

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_URDF = ROOT / "description" / "qmini_arm.urdf"


@dataclass(frozen=True)
class OfflineMotion:
    """Loaded FK, collision, and planning stack for one URDF."""

    model: ArmModel
    collision: CollisionChecker
    planner: MotionPlanner

    @classmethod
    def load(
        cls,
        urdf_path: str | Path = DEFAULT_URDF,
        *,
        planner_config: PlannerConfig | None = None,
    ) -> OfflineMotion:
        model = ArmModel(urdf_path)
        collision = CollisionChecker(model)
        return cls(
            model=model,
            collision=collision,
            planner=MotionPlanner(model, collision, config=planner_config),
        )


def write_joint_trajectory_csv(
    trajectory: TimedTrajectory,
    joint_names: tuple[str, ...],
    output: str | Path,
) -> Path:
    """Write an offline joint trajectory, without opening a hardware device."""
    expected_shape = (len(trajectory.times_s), len(joint_names))
    if (
        trajectory.positions_rad.shape != expected_shape
        or trajectory.velocities_rad_s.shape != expected_shape
    ):
        raise ValueError(f"trajectory positions and velocities must have shape {expected_shape}")
    destination = Path(output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    position_columns = [f"{name}_position_rad" for name in joint_names]
    velocity_columns = [f"{name}_velocity_rad_s" for name in joint_names]
    with destination.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time_s", *position_columns, *velocity_columns])
        for time_s, position, velocity in zip(
            trajectory.times_s,
            trajectory.positions_rad,
            trajectory.velocities_rad_s,
            strict=True,
        ):
            writer.writerow(
                [
                    f"{time_s:.6f}",
                    *(f"{value:.9f}" for value in position),
                    *(f"{value:.9f}" for value in velocity),
                ]
            )
    return destination
