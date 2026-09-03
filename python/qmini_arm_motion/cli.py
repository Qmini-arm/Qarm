"""Command line entry points for FK, workspace, planning and visualization."""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

import numpy as np

from .collision import CollisionChecker
from .commands import M8010CommandMapper
from .model import ArmModel
from .planner import MotionPlanner, PlannerConfig
from .transforms import rotation_to_rpy

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_URDF = PROJECT_ROOT / "urdf" / "qmini_arm.urdf"
DEFAULT_MOTOR_CONFIG = PROJECT_ROOT / "config" / "m8010_arm.yaml"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="qmini-motion",
        description="Qmini collision-aware FK/IK planning and M8010 command visualization",
    )
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--motor-config", type=Path, default=DEFAULT_MOTOR_CONFIG)
    parser.add_argument("-v", "--verbose", action="count", default=0)
    commands = parser.add_subparsers(dest="command", required=True)

    fk = commands.add_parser("fk", help="base_link -> tool0 forward kinematics")
    fk.add_argument("--q-deg", type=float, nargs=6, required=True)

    workspace = commands.add_parser("workspace", help="sample collision-free reachable points")
    workspace.add_argument("--samples", type=int, default=20000)
    workspace.add_argument("--seed", type=int, default=0)
    workspace.add_argument("--output", type=Path)

    plan = commands.add_parser("plan", help="plan from a joint state to a target point")
    plan.add_argument("--target", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"))
    plan.add_argument("--start-deg", type=float, nargs=6, default=[0.0] * 6)
    plan.add_argument("--output", type=Path, help="write M8010 command frames as CSV")

    viz = commands.add_parser("viz", help="launch the interactive browser simulation")
    viz.add_argument("--start-deg", type=float, nargs=6, default=[0.0] * 6)
    viz.add_argument("--workspace-samples", type=int, default=20000)
    viz.add_argument("--host", default="0.0.0.0")
    viz.add_argument("--port", type=int, default=8080)
    return parser


def _planner(
    model: ArmModel, collision: CollisionChecker, mapper: M8010CommandMapper
) -> MotionPlanner:
    return MotionPlanner(
        model,
        collision,
        config=PlannerConfig(
            velocity_limit_rad_s=mapper.joint_velocity_limit_rad_s,
            acceleration_limit_rad_s2=mapper.joint_acceleration_limit_rad_s2,
            control_period_s=mapper.control_period_s,
        ),
    )


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s: %(message)s",
    )
    try:
        model = ArmModel(args.urdf)
        collision = CollisionChecker(model)
        mapper = M8010CommandMapper.from_yaml(model, args.motor_config)

        if args.command == "fk":
            q = np.radians(args.q_deg)
            if not model.within_limits(q):
                raise ValueError("joint vector violates the URDF soft limits")
            pose = model.fk(q)
            print("position_m:", np.array2string(pose[:3, 3], precision=6))
            print(
                "rpy_deg:", np.array2string(np.degrees(rotation_to_rpy(pose[:3, :3])), precision=3)
            )
            print("self_collision:", ", ".join(map(str, collision.check(q))) or "none")
            return 0

        if args.command == "workspace":
            from .workspace import sample_workspace

            workspace = sample_workspace(model, collision, count=args.samples, seed=args.seed)
            lower, upper = workspace.bounds
            radii = np.linalg.norm(workspace.positions_m, axis=1)
            print(
                "samples: "
                f"{workspace.accepted_samples}/{workspace.requested_samples} collision-free"
            )
            print(
                f"bounds_m: x[{lower[0]:+.4f},{upper[0]:+.4f}] "
                f"y[{lower[1]:+.4f},{upper[1]:+.4f}] z[{lower[2]:+.4f},{upper[2]:+.4f}]"
            )
            print(f"radius_m: [{radii.min():.4f}, {radii.max():.4f}]")
            if args.output:
                print(f"saved: {workspace.save(args.output)}")
            return 0

        if args.command == "plan":
            start = np.radians(args.start_deg)
            motion = _planner(model, collision, mapper).plan(start, args.target)
            end = motion.trajectory.positions_rad[-1]
            print(f"status: {motion.ik.status.value}")
            print(
                f"path: {motion.path_kind}, waypoints={len(motion.waypoints_rad)}, "
                f"duration={motion.trajectory.duration_s:.3f}s"
            )
            print(f"position_error_mm: {motion.ik.position_error_m * 1000.0:.3f}")
            print("goal_joint_deg:", np.array2string(np.degrees(end), precision=3))
            final_frame = mapper.map_sample(
                motion.trajectory.duration_s,
                end,
                np.zeros(model.dof),
                start,
            )
            print("goal_motor_commands:")
            for motor in final_frame.motors:
                absolute = (
                    "uncalibrated"
                    if motor.rotor_position_rad is None
                    else f"{motor.rotor_position_rad:+.6f}"
                )
                print(
                    f"  id={motor.motor_id} {motor.joint_name}: q_rotor={absolute}, "
                    f"delta={motor.rotor_offset_from_start_rad:+.6f}, "
                    f"dq={motor.rotor_velocity_rad_s:+.6f}, kp={motor.kp_rotor:.3f}, "
                    f"kd={motor.kd_rotor:.3f}, tau_ff={motor.torque_ff_nm:+.3f}"
                )
            if args.output:
                print(f"saved: {mapper.write_csv(motion.trajectory, args.output)}")
            if not mapper.absolute_positions_available:
                print(
                    "WARNING: calibration placeholders prevent absolute rotor commands; "
                    "do not send this CSV to hardware.",
                    file=sys.stderr,
                )
            return 0

        if args.command == "viz":
            from .visualization import launch_visualization

            launch_visualization(
                model,
                collision,
                _planner(model, collision, mapper),
                mapper,
                initial_q=np.radians(args.start_deg),
                workspace_samples=args.workspace_samples,
                host=args.host,
                port=args.port,
            )
            return 0
    except (ValueError, RuntimeError, FileNotFoundError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
