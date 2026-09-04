from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
import sysconfig
import time
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image
from qmini_arm_motion import PlannerConfig, TimedTrajectory, sample_workspace
from qmini_arm_motion.transforms import rotation_to_rpy

from qarm_sim.calibration import estimate_zero, save_zero_calibration
from qarm_sim.calibration_pose import solve_table_supported_pose
from qarm_sim.config import DEFAULT_JOINT_MAP, JointMap, MotorParameters
from qarm_sim.home_simulation import add_endpoint_holds, simulate_home_trajectory
from qarm_sim.model import build_scene, set_mirrored_state
from qarm_sim.motion import DEFAULT_URDF, OfflineMotion, write_joint_trajectory_csv
from qarm_sim.telemetry import (
    SubprocessTelemetry,
    TelemetryError,
    map_joint_state,
)

DEFAULT_REMOTE_READER = "/home/HwHiAiUser/.local/libexec/qarm/m8010_readonly"
HOME_CONTROL_PERIOD_S = 0.01
HOME_VELOCITY_LIMIT_RAD_S = 0.25
HOME_ACCELERATION_LIMIT_RAD_S2 = 0.50
HOME_START_HOLD_S = 3.0
HOME_END_HOLD_S = 2.0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Qmini Arm MuJoCo, offline motion planning, and read-only telemetry"
    )
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("validate", help="compile and inspect the MuJoCo model")

    fk = commands.add_parser("fk", help="compute offline base_link-to-tool0 forward kinematics")
    _motion_model_arguments(fk)
    fk.add_argument("--q-deg", type=float, nargs=6, required=True)

    plan = commands.add_parser(
        "plan", help="plan an offline collision-free trajectory to a Cartesian point"
    )
    _motion_model_arguments(plan)
    plan.add_argument("--target", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"))
    plan.add_argument("--start-deg", type=float, nargs=6, default=[0.0] * 6)
    plan.add_argument("--output", type=Path, help="write the joint trajectory as CSV")

    plan_home = commands.add_parser(
        "plan-home", help="plan an offline collision-free trajectory to the URDF zero pose"
    )
    _motion_model_arguments(plan_home)
    plan_home.add_argument(
        "--start-deg",
        type=float,
        nargs=6,
        required=True,
        help="explicit current joint state in degrees; no hardware state is read",
    )
    plan_home.add_argument("--output", type=Path, help="write the joint trajectory as CSV")

    workspace = commands.add_parser(
        "workspace", help="sample offline collision-free FK workspace points"
    )
    _motion_model_arguments(workspace)
    workspace.add_argument("--samples", type=int, default=20000)
    workspace.add_argument("--seed", type=int, default=0)
    workspace.add_argument("--output", type=Path, help="write sampled points as NPZ")

    commands.add_parser(
        "solve-calibration-pose",
        help="recompute the manual reference pose from the current STL meshes",
    )
    viewer = commands.add_parser("viewer", help="open the calibrated CAD zero pose")
    viewer.add_argument("--duration", type=float, default=0.0)
    viewer_pose = viewer.add_mutually_exclusive_group()
    viewer_pose.add_argument("--calibration-pose", action="store_true")
    viewer_pose.add_argument("--joint-position", type=float, nargs=6)
    render = commands.add_parser("render", help="render the CAD zero pose to PNG")
    render.add_argument("--output", type=Path, default=Path("build/qarm-zero.png"))
    render.add_argument("--width", type=int, default=960)
    render.add_argument("--height", type=int, default=720)
    render_pose = render.add_mutually_exclusive_group()
    render_pose.add_argument("--calibration-pose", action="store_true")
    render_pose.add_argument(
        "--joint-position",
        type=float,
        nargs=6,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
    )

    inspect = commands.add_parser(
        "inspect-stream", help="print mapped joint states without opening a viewer"
    )
    _telemetry_arguments(inspect)
    inspect.add_argument("--samples", type=int, default=5)

    capture = commands.add_parser(
        "capture-zero",
        help="capture encoder zeros from the STL-derived manual reference pose",
    )
    _telemetry_arguments(capture)
    capture.add_argument("--samples", type=int, default=200)
    capture.add_argument("--maximum-span-rad", type=float, default=0.01)
    capture.add_argument(
        "--confirm-table-supported-pose",
        action="store_true",
        help=(
            "confirm the arm is in the accepted STL-derived pose, including "
            "motor ID 5 clockwise at its mechanical limit"
        ),
    )

    mirror = commands.add_parser("mirror", help="mirror live read-only motor feedback in MuJoCo")
    _telemetry_arguments(mirror)
    mirror.add_argument("--duration", type=float, default=0.0)
    mirror.add_argument("--status-rate", type=float, default=2.0)
    return parser


def _motion_model_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--urdf",
        type=Path,
        default=DEFAULT_URDF,
        help="expanded URDF used by the offline motion library",
    )


def _telemetry_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--ssh-target", default="HwHiAiUser@192.168.10.102")
    parser.add_argument("--device", default="/dev/ttyUSB0")
    parser.add_argument("--remote-reader", default=DEFAULT_REMOTE_READER)
    parser.add_argument("--joint-map", type=Path, default=DEFAULT_JOINT_MAP)
    parser.add_argument(
        "--record",
        type=Path,
        default=None,
        help="append raw remote NDJSON to this path",
    )
    parser.add_argument(
        "--acknowledge-supported-arm",
        action="store_true",
        help=(
            "confirm the arm is mechanically supported and allow the reader "
            "to send BRAKE/zero query frames"
        ),
    )


def _remote_command(args: argparse.Namespace) -> list[str]:
    acknowledge = " --acknowledge-state-change" if args.acknowledge_supported_arm else ""
    command = (
        f"{shlex.quote(args.remote_reader)} --device {shlex.quote(args.device)} "
        "--ids 0,1,2,3,4,5 --mode brake --rate 100"
        f"{acknowledge}"
    )
    return [
        "ssh",
        "-o",
        "BatchMode=yes",
        "-o",
        "ConnectTimeout=5",
        args.ssh_target,
        command,
    ]


def _remote_boot_id(ssh_target: str) -> str:
    completed = subprocess.run(
        [
            "ssh",
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout=5",
            ssh_target,
            "cat /proc/sys/kernel/random/boot_id",
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=8,
    )
    value = completed.stdout.strip()
    if completed.returncode != 0 or not value:
        message = completed.stderr.strip() or f"exit {completed.returncode}"
        raise TelemetryError(f"cannot read remote board boot ID: {message}")
    return value


def _verify_calibration_boot(mapping: JointMap, ssh_target: str) -> None:
    expected = mapping.calibration_board_boot_id
    if not mapping.zero_calibrated or expected is None:
        return
    actual = _remote_boot_id(ssh_target)
    if actual != expected:
        raise TelemetryError(
            "saved zero belongs to a different board boot; recapture the URDF "
            "zero after every board or motor power cycle"
        )


def _mjpython_executable() -> Path:
    executable = Path(sys.executable).with_name("mjpython")
    if not executable.is_file():
        raise RuntimeError(f"mjpython executable not found: {executable}")
    return executable


def _mjpython_environment() -> dict[str, str]:
    environment = os.environ.copy()
    fallback = []
    libdir = sysconfig.get_config_var("LIBDIR")
    if libdir:
        fallback.append(str(libdir))
    existing = environment.get("DYLD_FALLBACK_LIBRARY_PATH")
    if existing:
        fallback.extend(value for value in existing.split(os.pathsep) if value)
    fallback.extend(("/usr/local/lib", "/usr/lib"))
    environment["DYLD_FALLBACK_LIBRARY_PATH"] = os.pathsep.join(dict.fromkeys(fallback))
    return environment


def _relaunch_viewer_if_needed(arguments: list[str]) -> int | None:
    if sys.platform != "darwin":
        return None
    import mujoco.viewer

    if mujoco.viewer._MJPYTHON is not None:
        return None
    return subprocess.run(
        [str(_mjpython_executable()), "-m", "qarm_sim.cli", *arguments],
        env=_mjpython_environment(),
        check=False,
    ).returncode


def command_validate() -> int:
    scene = build_scene()
    mujoco.mj_forward(scene.model, scene.data)
    collision_geoms = int(np.count_nonzero(scene.model.geom_contype))
    visual_geoms = int(np.count_nonzero(scene.model.geom_contype == 0))
    result = {
        "mujoco_version": mujoco.__version__,
        "nq": scene.model.nq,
        "nv": scene.model.nv,
        "nu": scene.model.nu,
        "nbody": scene.model.nbody,
        "ngeom": scene.model.ngeom,
        "nmesh": scene.model.nmesh,
        "nsite": scene.model.nsite,
        "nsensor": scene.model.nsensor,
        "collision_geoms": collision_geoms,
        "visual_geoms": visual_geoms,
        "joint_names": list(scene.joint_names),
        "joint_range_rad": scene.model.jnt_range[scene.joint_ids].tolist(),
        "zero_gravity_bias_nm": scene.data.qfrc_bias[scene.dof_addresses].tolist(),
        "tool0_position_m": scene.data.site_xpos[scene.tool_site_id].tolist(),
        "identified_motor_parameters_complete": False,
    }
    print(json.dumps(result, indent=2))
    return 0


def _offline_result(**values: object) -> dict[str, object]:
    return {
        "execution": "offline_only",
        "hardware_io_performed": False,
        "collision_scope": "URDF self-collision only; environment and cables are excluded",
        "trajectory_semantics": "offline joint references; not hardware motor commands",
        **values,
    }


def command_fk(args: argparse.Namespace) -> int:
    motion = OfflineMotion.load(args.urdf)
    position = np.radians(np.asarray(args.q_deg, dtype=np.float64))
    if not motion.model.within_limits(position):
        raise ValueError("joint vector violates the URDF soft limits")
    pose = motion.model.fk(position)
    collisions = motion.collision.check(position)
    print(
        json.dumps(
            _offline_result(
                frame="base_link",
                joint_names=list(motion.model.joint_names),
                joint_position_rad=position.tolist(),
                tool0_position_m=pose[:3, 3].tolist(),
                tool0_rpy_rad=rotation_to_rpy(pose[:3, :3]).tolist(),
                self_collision=[str(pair) for pair in collisions],
            ),
            indent=2,
        )
    )
    return 0


def _trajectory_summary(
    trajectory: TimedTrajectory,
    *,
    output: Path | None,
    joint_names: tuple[str, ...],
) -> dict[str, object]:
    saved = (
        write_joint_trajectory_csv(trajectory, joint_names, output).resolve()
        if output is not None
        else None
    )
    return {
        "duration_s": trajectory.duration_s,
        "samples": len(trajectory.times_s),
        "maximum_joint_speed_rad_s": float(np.max(np.abs(trajectory.velocities_rad_s))),
        "trajectory_csv": None if saved is None else str(saved),
    }


def command_plan(args: argparse.Namespace) -> int:
    motion = OfflineMotion.load(args.urdf)
    start = np.radians(np.asarray(args.start_deg, dtype=np.float64))
    target = np.asarray(args.target, dtype=np.float64)
    plan = motion.planner.plan(start, target)
    result = _offline_result(
        frame="base_link",
        joint_names=list(motion.model.joint_names),
        start_joint_position_rad=start.tolist(),
        target_position_m=target.tolist(),
        goal_joint_position_rad=plan.trajectory.positions_rad[-1].tolist(),
        ik_status=plan.ik.status.value,
        position_error_m=plan.ik.position_error_m,
        path_kind=plan.path_kind,
        waypoints=len(plan.waypoints_rad),
        **_trajectory_summary(
            plan.trajectory,
            output=args.output,
            joint_names=motion.model.joint_names,
        ),
    )
    print(json.dumps(result, indent=2))
    return 0


def command_plan_home(args: argparse.Namespace) -> int:
    motion = OfflineMotion.load(
        args.urdf,
        planner_config=PlannerConfig(
            velocity_limit_rad_s=HOME_VELOCITY_LIMIT_RAD_S,
            acceleration_limit_rad_s2=HOME_ACCELERATION_LIMIT_RAD_S2,
            control_period_s=HOME_CONTROL_PERIOD_S,
        ),
    )
    start = np.radians(np.asarray(args.start_deg, dtype=np.float64))
    plan = motion.planner.plan_home(start)
    trajectory = add_endpoint_holds(
        plan.trajectory,
        control_period_s=HOME_CONTROL_PERIOD_S,
        start_hold_s=HOME_START_HOLD_S,
        end_hold_s=HOME_END_HOLD_S,
    )
    mapping = JointMap.load()
    simulation = simulate_home_trajectory(
        build_scene(mapping=mapping),
        trajectory,
        directions=mapping.direction,
    )
    if not simulation.passed:
        raise RuntimeError(
            "MuJoCo return-to-zero experiment failed; trajectory was not exported; "
            f"max_speed={np.max(simulation.maximum_speed_rad_s):.3f} rad/s, "
            f"max_tracking_error={np.max(simulation.maximum_tracking_error_rad):.3f} rad, "
            f"contacts={simulation.contact_steps}, "
            f"torque_saturation_steps={simulation.torque_saturation_steps}"
        )
    result = _offline_result(
        frame="base_link",
        joint_names=list(motion.model.joint_names),
        start_joint_position_rad=start.tolist(),
        goal_joint_position_rad=plan.goal_position_rad.tolist(),
        goal="URDF zero configuration",
        path_kind=plan.path_kind,
        waypoints=len(plan.waypoints_rad),
        planner_velocity_limit_rad_s=HOME_VELOCITY_LIMIT_RAD_S,
        planner_acceleration_limit_rad_s2=HOME_ACCELERATION_LIMIT_RAD_S2,
        control_period_s=HOME_CONTROL_PERIOD_S,
        start_hold_s=HOME_START_HOLD_S,
        end_hold_s=HOME_END_HOLD_S,
        mujoco_simulation=simulation.as_dict(),
        **_trajectory_summary(
            trajectory,
            output=args.output,
            joint_names=motion.model.joint_names,
        ),
    )
    print(json.dumps(result, indent=2))
    return 0


def command_workspace(args: argparse.Namespace) -> int:
    motion = OfflineMotion.load(args.urdf)
    sampled = sample_workspace(
        motion.model,
        motion.collision,
        count=args.samples,
        seed=args.seed,
    )
    saved = sampled.save(args.output).resolve() if args.output is not None else None
    bounds = sampled.bounds if sampled.accepted_samples else None
    print(
        json.dumps(
            _offline_result(
                frame="base_link",
                requested_samples=sampled.requested_samples,
                accepted_samples=sampled.accepted_samples,
                collision_free_fraction=sampled.collision_free_fraction,
                bounds_m=None
                if bounds is None
                else {"lower": bounds[0].tolist(), "upper": bounds[1].tolist()},
                workspace_npz=None if saved is None else str(saved),
            ),
            indent=2,
        )
    )
    return 0


def command_solve_calibration_pose() -> int:
    solution = solve_table_supported_pose()
    print(
        json.dumps(
            {
                "name": "table_supported_stl_v1",
                "joint_position_rad": solution.joint_position_rad.tolist(),
                "joint_position_deg": solution.joint_position_deg.tolist(),
                "motor_ids_by_joint": [0, 1, 2, 3, 4, 5],
                "table_z_m": solution.table_z_m,
                "arm_link_gap_m": solution.arm_link_gap_m,
                "motor_3_gap_m": solution.motor_3_gap_m,
                "motor_4_face_horizontal_error_deg": (solution.motor_4_axis_vertical_error_deg),
                "motor_5_table_clearance_m": (solution.motor_5_table_clearance_m),
                "motor_5_reference": ("clockwise mechanical limit = joint_6 lower limit"),
                "manual_calibration_only": True,
            },
            indent=2,
        )
    )
    return 0


def command_render(args: argparse.Namespace) -> int:
    scene = build_scene()
    position = (
        solve_table_supported_pose(scene).joint_position_rad
        if args.calibration_pose
        else np.asarray(args.joint_position or [0.0] * 6, dtype=np.float64)
    )
    set_mirrored_state(scene, position)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with mujoco.Renderer(scene.model, height=args.height, width=args.width) as renderer:
        renderer.update_scene(scene.data, camera="overview")
        pixels = renderer.render()
    Image.fromarray(pixels).save(args.output)
    print(args.output.resolve())
    return 0


def command_viewer(args: argparse.Namespace) -> int:
    import mujoco.viewer

    scene = build_scene()
    position = (
        solve_table_supported_pose(scene).joint_position_rad
        if args.calibration_pose
        else np.asarray(args.joint_position or [0.0] * 6, dtype=np.float64)
    )
    set_mirrored_state(scene, position)
    started = time.monotonic()
    with mujoco.viewer.launch_passive(scene.model, scene.data) as handle:
        while handle.is_running():
            if args.duration > 0 and time.monotonic() - started >= args.duration:
                break
            handle.sync()
            time.sleep(1.0 / 60.0)
    return 0


def command_inspect_stream(args: argparse.Namespace) -> int:
    mapping = JointMap.load(args.joint_map)
    _verify_calibration_boot(mapping, args.ssh_target)
    count = 0
    with SubprocessTelemetry(_remote_command(args), record_path=args.record) as telemetry:
        while count < args.samples:
            sample = telemetry.latest(timeout=3.0)
            state = map_joint_state(sample, mapping)
            print(
                json.dumps(
                    {
                        "sequence": sample.sequence,
                        "joint_position_rad": state.position.tolist(),
                        "joint_velocity_rad_s": state.velocity.tolist(),
                        "joint_torque_nm": state.torque.tolist(),
                        "temperature_c": state.temperature_c.tolist(),
                        "motor_error": state.motor_error.tolist(),
                        "mapping_calibrated": mapping.calibrated,
                        "zero_calibrated": mapping.zero_calibrated,
                        "direction_calibrated": mapping.direction_calibrated,
                    }
                )
            )
            count += 1
    return 0


def command_capture_zero(args: argparse.Namespace) -> int:
    if not args.confirm_table_supported_pose:
        raise ValueError(
            "refusing to capture: place the arm in the accepted table-supported "
            "pose, turn motor ID 5 clockwise to its mechanical limit, and pass "
            "--confirm-table-supported-pose"
        )
    if not args.acknowledge_supported_arm:
        raise ValueError(
            "refusing to open the bus: mechanically support the arm and pass "
            "--acknowledge-supported-arm"
        )
    if args.samples < 20 or args.samples > 5000:
        raise ValueError("--samples must be within 20..5000")
    mapping = JointMap.load(args.joint_map)
    motor = MotorParameters.load()
    reference = solve_table_supported_pose().joint_position_rad
    board_boot_id = _remote_boot_id(args.ssh_target)
    samples = []
    with SubprocessTelemetry(_remote_command(args), record_path=args.record) as telemetry:
        while len(samples) < args.samples:
            samples.append(telemetry.latest(timeout=3.0))
    estimate = estimate_zero(
        samples,
        mapping,
        maximum_span_rad=args.maximum_span_rad,
        reference_joint_rad=reference,
        gear_ratio=motor.gear_ratio,
    )
    backup = save_zero_calibration(
        args.joint_map,
        mapping,
        estimate,
        board_boot_id=board_boot_id,
    )
    print(
        json.dumps(
            {
                "success": True,
                "joint_map": str(args.joint_map.resolve()),
                "backup": str(backup.resolve()),
                "board_boot_id": board_boot_id,
                "samples": estimate.samples,
                "source_at_reference_rad": (estimate.source_at_reference_rad.tolist()),
                "rotor_at_reference_rad": (estimate.rotor_at_reference_rad.tolist()),
                "source_zero_rad": estimate.source_zero_rad.tolist(),
                "rotor_zero_rad": estimate.rotor_zero_rad.tolist(),
                "reference_joint_rad": reference.tolist(),
                "reference_joint_deg": np.degrees(reference).tolist(),
                "position_span_rad": estimate.position_span_rad.tolist(),
                "direction_still_requires_verification": True,
                "motion_config_was_not_enabled": True,
            },
            indent=2,
        )
    )
    return 0


def command_mirror(args: argparse.Namespace) -> int:
    import mujoco.viewer

    mapping = JointMap.load(args.joint_map)
    _verify_calibration_boot(mapping, args.ssh_target)
    motor = MotorParameters.load()
    scene = build_scene(mapping=mapping, motor=motor)
    record = args.record
    started = time.monotonic()
    last_status = float("-inf")
    last_sequence = -1
    last_sample_at = started
    stale_reported = False
    with SubprocessTelemetry(_remote_command(args), record_path=record) as telemetry:
        first = telemetry.latest(timeout=5.0)
        state = map_joint_state(first, mapping)
        set_mirrored_state(scene, state.position, state.velocity)
        with mujoco.viewer.launch_passive(
            scene.model, scene.data, show_left_ui=False, show_right_ui=True
        ) as handle:
            while handle.is_running():
                now = time.monotonic()
                if args.duration > 0 and now - started >= args.duration:
                    break
                try:
                    sample = telemetry.latest(timeout=0.02)
                except TelemetryError as error:
                    if "timed out" not in str(error):
                        raise
                else:
                    state = map_joint_state(sample, mapping)
                    set_mirrored_state(scene, state.position, state.velocity)
                    last_sequence = sample.sequence
                    last_sample_at = now
                    stale_reported = False
                if now - last_sample_at > 0.5 and not stale_reported:
                    print(
                        f"STALE: no valid motor sample for {now - last_sample_at:.3f}s",
                        flush=True,
                    )
                    stale_reported = True
                if now - last_status >= 1.0 / args.status_rate:
                    position_text = np.array2string(state.position, precision=4)
                    print(
                        f"seq={last_sequence} calibrated={mapping.calibrated} "
                        f"q={position_text} "
                        f"temp={state.temperature_c.tolist()} "
                        f"err={state.motor_error.tolist()} record={record}",
                        flush=True,
                    )
                    last_status = now
                handle.sync()
    return 0


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    args = _parser().parse_args(argv)
    try:
        if args.command == "validate":
            return command_validate()
        if args.command == "fk":
            return command_fk(args)
        if args.command == "plan":
            return command_plan(args)
        if args.command == "plan-home":
            return command_plan_home(args)
        if args.command == "workspace":
            return command_workspace(args)
        if args.command == "solve-calibration-pose":
            return command_solve_calibration_pose()
        if args.command == "render":
            return command_render(args)
        if args.command == "viewer":
            relaunched = _relaunch_viewer_if_needed(argv)
            if relaunched is not None:
                return relaunched
            return command_viewer(args)
        if args.command == "inspect-stream":
            return command_inspect_stream(args)
        if args.command == "capture-zero":
            return command_capture_zero(args)
        if args.command == "mirror":
            relaunched = _relaunch_viewer_if_needed(argv)
            if relaunched is not None:
                return relaunched
            return command_mirror(args)
    except (FileNotFoundError, ValueError, RuntimeError, TelemetryError) as error:
        print(f"qarm-sim: {error}", file=sys.stderr)
        return 2
    raise AssertionError(args.command)


if __name__ == "__main__":
    raise SystemExit(main())
