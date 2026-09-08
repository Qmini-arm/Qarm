"""Command-line entry point for the Qarm MuJoCo environment."""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np

from motor_driver import MotorCmd, MotorData, MujocoSerialPort

from .env import DEFAULT_MODEL_PATH, JOINT_NAMES, QArmMujocoEnv
from .gravity_compare import (
    DEFAULT_PI_COEFFICIENTS,
    compare_gravity_compensation,
)
from .viser_app import run_viser


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Qarm four-axis MuJoCo dynamics and M8010-compatible control"
    )
    commands = parser.add_subparsers(dest="command", required=True)

    validate = commands.add_parser("validate", help="compile and inspect robot.xml")
    validate.add_argument("--model", type=Path, default=DEFAULT_MODEL_PATH)

    gravity = commands.add_parser(
        "compare-gravity",
        help="compare MuJoCo inverse-dynamics gravity with the legacy PI formula",
    )
    gravity.add_argument("--model", type=Path, default=DEFAULT_MODEL_PATH)
    gravity.add_argument(
        "--q",
        type=float,
        nargs=4,
        default=(0.0, 0.8, 0.2, 0.0),
        metavar=("J1", "J2", "J3", "J4"),
        help="joint position in radians",
    )
    gravity.add_argument(
        "--pi",
        type=float,
        nargs=3,
        default=DEFAULT_PI_COEFFICIENTS,
        metavar=("PI1", "PI2", "PI3"),
        help="legacy empirical coefficients in N m",
    )

    viser_command = commands.add_parser(
        "viser", aliases=["mjviser"], help="open the Viser FK/IK workbench"
    )
    viser_command.add_argument("--model", type=Path, default=DEFAULT_MODEL_PATH)
    viser_command.add_argument("--host", default="127.0.0.1")
    viser_command.add_argument("--port", type=int, default=8080)

    demo = commands.add_parser(
        "demo",
        help="run joint-space M8010 PD control with optional gravity feed-forward",
    )
    demo.add_argument("--model", type=Path, default=DEFAULT_MODEL_PATH)
    demo.add_argument("--duration", type=float, default=10.0)
    demo.add_argument(
        "--initial",
        type=float,
        nargs=4,
        default=(0.0, 0.8, 0.2, 0.0),
        metavar=("J1", "J2", "J3", "J4"),
        help="initial joint position in radians",
    )
    demo.add_argument(
        "--target",
        type=float,
        nargs=4,
        default=None,
        metavar=("J1", "J2", "J3", "J4"),
        help="target joint position in radians (default: initial position)",
    )
    demo.add_argument(
        "--kp",
        type=float,
        nargs="+",
        default=(0.2,),
        help="one or four M8010 rotor-side proportional gains",
    )
    demo.add_argument(
        "--kd",
        type=float,
        nargs="+",
        default=(0.03,),
        help="one or four M8010 rotor-side derivative gains",
    )
    demo.add_argument(
        "--control-rate",
        type=float,
        default=200.0,
        help="host command update rate in Hz",
    )
    demo.add_argument(
        "--no-gravity-compensation",
        action="store_true",
        help="disable ideal MuJoCo gravity feed-forward",
    )
    demo.add_argument(
        "--sine-joint",
        type=int,
        choices=range(1, 5),
        default=None,
        metavar="1..4",
        help="add a sine target to one joint",
    )
    demo.add_argument("--amplitude", type=float, default=0.25, help="sine amplitude, rad")
    demo.add_argument("--period", type=float, default=4.0, help="sine period, seconds")
    demo.add_argument(
        "--headless",
        action="store_true",
        help="run deterministically without the interactive viewer",
    )
    return parser


def _four_values(values: list[float] | tuple[float, ...], name: str) -> np.ndarray:
    result = np.asarray(values, dtype=np.float64)
    if result.shape == (1,):
        result = np.repeat(result, len(JOINT_NAMES))
    if result.shape != (len(JOINT_NAMES),) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} requires one or four finite values")
    if np.any(result < 0.0):
        raise ValueError(f"{name} values must be non-negative")
    return result


def _validate_demo_args(args: argparse.Namespace) -> None:
    positive = (args.duration, args.control_rate, args.period)
    if not all(math.isfinite(value) and value > 0.0 for value in positive):
        raise ValueError("duration, control rate, and sine period must be positive")
    if not math.isfinite(args.amplitude) or args.amplitude < 0.0:
        raise ValueError("sine amplitude must be finite and non-negative")


def command_validate(args: argparse.Namespace) -> int:
    with QArmMujocoEnv(args.model) as environment:
        print(json.dumps(environment.model_info(), indent=2))
    return 0


def command_compare_gravity(args: argparse.Namespace) -> int:
    with QArmMujocoEnv(args.model) as environment:
        comparison = compare_gravity_compensation(
            environment,
            args.q,
            args.pi,
        )
    print(json.dumps(comparison.as_dict(), indent=2))
    return 0


def _update_commands(
    bus: MujocoSerialPort,
    commands: list[MotorCmd],
    feedback: list[MotorData],
    target: np.ndarray,
    elapsed_s: float,
    *,
    sine_joint: int | None,
    amplitude: float,
    period_s: float,
    gravity_compensation: bool,
) -> np.ndarray:
    simulation = bus.simulation
    if simulation is None:
        raise RuntimeError("demo requires the MuJoCo transport")
    current_target = target.copy()
    if sine_joint is not None:
        current_target[sine_joint - 1] += amplitude * math.sin(
            2.0 * math.pi * elapsed_s / period_s
        )
    feedforward = (
        simulation.gravity_compensation()
        if gravity_compensation
        else np.zeros(len(JOINT_NAMES), dtype=np.float64)
    )
    for index, (command, state) in enumerate(zip(commands, feedback)):
        command.q = float(current_target[index])
        command.dq = 0.0
        command.tau = float(feedforward[index])
        if not bus.sendRecv(command, state):
            raise RuntimeError(f"simulated M8010 command failed for motor {index}")
    return current_target


def _summary(
    bus: MujocoSerialPort,
    target: np.ndarray,
    maximum_error: np.ndarray,
) -> dict[str, object]:
    simulation = bus.simulation
    if simulation is None:
        raise RuntimeError("demo requires the MuJoCo transport")
    state = simulation.snapshot()
    return {
        "execution": "offline_mujoco_only",
        "simulated_time_s": state.time_s,
        "joint_names": list(JOINT_NAMES),
        "target_position_rad": target.tolist(),
        "final_position_rad": state.position_rad.tolist(),
        "final_error_rad": (target - state.position_rad).tolist(),
        "maximum_tracking_error_rad": maximum_error.tolist(),
        "maximum_abs_velocity_rad_s": np.abs(state.velocity_rad_s).tolist(),
        "actuator_torque_nm": state.actuator_torque_nm.tolist(),
        "saturation_steps": simulation.saturation_steps,
        "command_clipped": state.command_clipped.tolist(),
        "tool_position_m": state.tool_position_m.tolist(),
        "active_contacts": [list(pair) for pair in state.contacts],
        "temperature_and_faults_modeled": False,
        "hardware_io_performed": False,
    }


def _headless_demo(args: argparse.Namespace) -> dict[str, object]:
    initial = np.asarray(args.initial, dtype=np.float64)
    target = initial.copy() if args.target is None else np.asarray(args.target, dtype=np.float64)
    kp = _four_values(args.kp, "--kp")
    kd = _four_values(args.kd, "--kd")
    bus = MujocoSerialPort(args.model, realtime=False, initial_qpos=initial)
    try:
        simulation = bus.simulation
        assert simulation is not None
        commands = [
            MotorCmd(id=index, mode=1, q=float(target[index]), kp=kp[index], kd=kd[index])
            for index in range(len(JOINT_NAMES))
        ]
        feedback = [MotorData() for _ in JOINT_NAMES]
        control_period = 1.0 / args.control_rate
        physics_steps = max(1, round(control_period / simulation.timestep))
        maximum_error = np.zeros(len(JOINT_NAMES), dtype=np.float64)
        last_target = target.copy()
        while simulation.data.time < args.duration - 0.5 * simulation.timestep:
            last_target = _update_commands(
                bus,
                commands,
                feedback,
                target,
                simulation.data.time,
                sine_joint=args.sine_joint,
                amplitude=args.amplitude,
                period_s=args.period,
                gravity_compensation=not args.no_gravity_compensation,
            )
            state = simulation.step(physics_steps)
            maximum_error = np.maximum(
                maximum_error, np.abs(last_target - state.position_rad)
            )
        return _summary(bus, last_target, maximum_error)
    finally:
        bus.close()


def _viewer_demo(args: argparse.Namespace) -> dict[str, object]:
    if sys.platform == "darwin":
        import mujoco.viewer

        if mujoco.viewer._MJPYTHON is None:
            raise RuntimeError(
                "macOS viewer must run through mjpython; use "
                "'.venv/bin/mjpython -m qarm_sim demo ...'"
            )

    initial = np.asarray(args.initial, dtype=np.float64)
    target = initial.copy() if args.target is None else np.asarray(args.target, dtype=np.float64)
    kp = _four_values(args.kp, "--kp")
    kd = _four_values(args.kd, "--kd")
    bus = MujocoSerialPort(args.model, realtime=True, initial_qpos=initial)
    stop = threading.Event()
    maximum_error = np.zeros(len(JOINT_NAMES), dtype=np.float64)
    last_target = target.copy()
    commands = [
        MotorCmd(id=index, mode=1, q=float(target[index]), kp=kp[index], kd=kd[index])
        for index in range(len(JOINT_NAMES))
    ]
    feedback = [MotorData() for _ in JOINT_NAMES]
    started = time.monotonic()

    def control_loop() -> None:
        nonlocal last_target, maximum_error
        period = 1.0 / args.control_rate
        next_update = time.monotonic()
        while not stop.is_set():
            elapsed = time.monotonic() - started
            last_target = _update_commands(
                bus,
                commands,
                feedback,
                target,
                elapsed,
                sine_joint=args.sine_joint,
                amplitude=args.amplitude,
                period_s=args.period,
                gravity_compensation=not args.no_gravity_compensation,
            )
            state = bus.simulation.snapshot()
            maximum_error = np.maximum(
                maximum_error, np.abs(last_target - state.position_rad)
            )
            next_update += period
            stop.wait(max(0.0, next_update - time.monotonic()))

    controller = threading.Thread(target=control_loop, name="qarm-demo-control", daemon=True)
    controller.start()
    try:
        bus.simulation.launch_viewer(duration_s=args.duration)
        return _summary(bus, last_target, maximum_error)
    finally:
        stop.set()
        controller.join(timeout=2.0)
        bus.close()


def command_demo(args: argparse.Namespace) -> int:
    _validate_demo_args(args)
    result = _headless_demo(args) if args.headless else _viewer_demo(args)
    print(json.dumps(result, indent=2))
    return 0


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "validate":
            return command_validate(args)
        if args.command == "compare-gravity":
            return command_compare_gravity(args)
        if args.command in ("viser", "mjviser"):
            run_viser(model_path=args.model, host=args.host, port=args.port)
            return 0
        if args.command == "demo":
            return command_demo(args)
        raise AssertionError(f"unhandled command {args.command!r}")
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
