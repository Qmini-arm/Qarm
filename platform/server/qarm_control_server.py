#!/usr/bin/env python3
"""Qarm control-plane HTTP service.

The browser talks to this process; it never opens ``/dev/ttyUSB0`` itself. The
default is deterministic simulation. Hardware mode is capability-gated: until
a real controller adapter is installed, requests that would energise or command
motors return ``501`` and the state is not changed.
"""

from __future__ import annotations

import csv
import json
import math
import os
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from collections.abc import Callable
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
PORT = int(os.environ.get("QARM_PLATFORM_PORT", "8090"))
HARDWARE = os.environ.get("QARM_HARDWARE", "0") == "1"
READER = os.environ.get("QARM_READER", "/home/HwHiAiUser/.local/libexec/qarm/m8010_readonly")
RETURN_HOME = os.environ.get("QARM_RETURN_HOME", "/home/HwHiAiUser/.local/bin/qmini-return-home")
GRAVITY = os.environ.get("QARM_GRAVITY", "/home/HwHiAiUser/.local/bin/qmini-gravity")
CONFIG = Path(os.environ.get("QARM_CONFIG", str(ROOT / "config" / "joint_map.json")))
URDF = Path(os.environ.get("QARM_URDF", str(ROOT / "description" / "qmini_arm.urdf")))
CALIBRATION_POSE = Path(
    os.environ.get("QARM_CALIBRATION_POSE", str(ROOT / "config" / "calibration_pose.json"))
)
DIST = Path(os.environ.get("QARM_PLATFORM_DIST", str(ROOT / "platform" / "dist")))


def load_model(urdf_path: Path, config_path: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Load the active serial chain and its exact hardware mapping."""
    root = ET.parse(urdf_path).getroot()
    by_child = {joint.find("child").get("link"): joint for joint in root.findall("joint")}
    chain = []
    current = "tool0"
    while current != "base_link":
        joint = by_child[current]
        if joint.get("type") != "fixed":
            chain.append(joint)
        current = joint.find("parent").get("link")
    chain.reverse()
    mapping = json.loads(config_path.read_text(encoding="utf-8"))
    names = [joint.get("name") for joint in chain]
    if mapping.get("joint_names") != names:
        raise ValueError("joint_map joint_names must exactly match the active URDF chain")
    ids = mapping.get("motor_ids_by_joint", [])
    if (
        len(ids) != len(names)
        or any(type(value) is not int or value < 0 or value > 14 for value in ids)
        or len(set(ids)) != len(ids)
    ):
        raise ValueError("joint_map must contain one unique motor ID in [0,14] per active joint")
    for field in ("direction", "zero_offset_rad"):
        values = mapping.get(field, [])
        if len(values) != len(names) or not all(
            type(value) in (int, float) and math.isfinite(value) for value in values
        ):
            raise ValueError(f"joint_map {field} must match the active joint count")
    if any(value not in (-1, 1) for value in mapping["direction"]):
        raise ValueError("joint_map direction values must be -1 or 1")
    calibration = mapping.get("calibration", {})
    if not isinstance(calibration, dict):
        raise ValueError("joint_map calibration must be an object")
    anchor_fields = ("reference_joint_rad", "source_at_reference_rad")
    if any(field in calibration for field in anchor_fields):
        for field in anchor_fields:
            values = calibration.get(field)
            if (
                not isinstance(values, list)
                or len(values) != len(names)
                or not all(type(value) in (int, float) and math.isfinite(value) for value in values)
            ):
                raise ValueError(f"joint_map calibration.{field} must match the active joint count")
    limits = []
    for joint in chain:
        hard = joint.find("limit")
        soft = joint.find("safety_controller")
        lower, upper = float(hard.get("lower")), float(hard.get("upper"))
        if soft is not None:
            lower = max(lower, float(soft.get("soft_lower_limit", str(lower))))
            upper = min(upper, float(soft.get("soft_upper_limit", str(upper))))
        if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
            raise ValueError(f"invalid active limits for {joint.get('name')}")
        limits.append((lower, upper))
    return [
        {
            "name": f"J{i + 1}",
            "joint_name": joint.get("name"),
            "id": ids[i],
            "angle": 0.0,
            "velocity": 0.0,
            "torque": 0.0,
            "temperature": 30.0,
            "error": 0,
            "min": limits[i][0],
            "max": limits[i][1],
        }
        for i, joint in enumerate(chain)
    ], mapping


def read_trajectory(path: str, joint_names: list[str]) -> list[float]:
    expected = [
        "time_s",
        *(f"{name}_position_rad" for name in joint_names),
        *(f"{name}_velocity_rad_s" for name in joint_names),
    ]
    with Path(path).open(newline="", encoding="utf-8") as stream:
        reader = csv.reader(stream)
        if next(reader, None) != expected:
            raise ActionError("trajectory columns must exactly match the active joint names", 400)
        final = None
        previous_time = -1.0
        for row in reader:
            try:
                values = [float(value) for value in row]
            except ValueError as exc:
                raise ActionError("trajectory values must be numeric", 400) from exc
            if len(values) != len(expected) or not all(math.isfinite(value) for value in values):
                raise ActionError(
                    "trajectory rows must contain finite values for every column", 400
                )
            if values[0] < 0 or values[0] <= previous_time:
                raise ActionError(
                    "trajectory times must be non-negative and strictly increasing", 400
                )
            previous_time = values[0]
            final = values[1 : 1 + len(joint_names)]
        if final is None:
            raise ActionError("trajectory must contain at least one sample", 400)
        return final


class ActionError(Exception):
    """An expected API action rejection with an HTTP status code."""

    def __init__(self, message: str, status: int = 409) -> None:
        super().__init__(message)
        self.status = status


class ControlState:
    """Thread-safe state machine and process owner for the control service."""

    def __init__(
        self,
        *,
        hardware: bool = HARDWARE,
        reader_path: str = READER,
        return_home_path: str = RETURN_HOME,
        gravity_path: str = GRAVITY,
        config_path: Path = CONFIG,
        urdf_path: Path = URDF,
        calibration_pose_path: Path = CALIBRATION_POSE,
        popen: Callable[..., subprocess.Popen[str]] = subprocess.Popen,
    ) -> None:
        self.lock = threading.RLock()
        self.hardware = bool(hardware)
        self.reader_path, self.return_home_path, self.gravity_path = (
            reader_path,
            return_home_path,
            gravity_path,
        )
        self._popen = popen
        self.connected = not self.hardware
        self.enabled = self.estop = self.gravity = False
        self.mode = "hardware" if self.hardware else "simulation"
        self.lifecycle = "disconnected" if self.hardware else "simulation_ready"
        self.config_path, self.urdf_path = Path(config_path), Path(urdf_path)
        self.joints, self.mapping = load_model(self.urdf_path, self.config_path)
        self.joint_names = [joint["joint_name"] for joint in self.joints]
        self.dof = len(self.joints)
        self.calibrated = all(
            self.mapping.get(key) is True
            for key in ("calibrated", "zero_calibrated", "direction_calibrated")
        )
        pose = json.loads(Path(calibration_pose_path).read_text(encoding="utf-8"))
        reference = pose.get("reference_joint_rad")
        self.home_reference = (
            reference
            if pose.get("validated") is True
            and pose.get("joint_names") == self.joint_names
            and isinstance(reference, list)
            and len(reference) == self.dof
            and all(type(value) in (int, float) and math.isfinite(value) for value in reference)
            else None
        )
        self.last_action = ""
        self.notice = "等待连接" if self.hardware else "仿真模式就绪"
        self.events: list[dict[str, Any]] = []
        self.reader: subprocess.Popen[str] | None = None
        self.reader_thread: threading.Thread | None = None
        self.motion_process: subprocess.Popen[str] | None = None
        self.gravity_process: subprocess.Popen[str] | None = None
        self._event("server_started", self.notice)

    def _event(self, action: str, message: str) -> None:
        self.events.append({"time": time.time(), "action": action, "message": message})
        self.events = self.events[-100:]

    def _set_action(self, action: str, message: str) -> None:
        self.last_action, self.notice = action, message
        self._event(action, message)

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {
                "connected": self.connected,
                "enabled": self.enabled,
                "estop": self.estop,
                "gravity": self.gravity,
                "mode": self.mode,
                "lifecycle": self.lifecycle,
                "joints": [dict(j) for j in self.joints],
                "dof": self.dof,
                "joint_names": list(self.joint_names),
                "calibrated": self.calibrated,
                "calibration_pose_validated": self.home_reference is not None,
                "angle_space": "urdf"
                if not self.hardware or self.calibrated
                else "uncalibrated_motor_output",
                "last_action": self.last_action,
                "notice": self.notice,
                "hardware_io_enabled": self.hardware,
                "reader": self.reader_path if self.hardware else None,
                "capabilities": {
                    "feedback": self.hardware,
                    "enable": not self.hardware,
                    "gravity": not self.hardware,
                    "movej": not self.hardware,
                    "return_home": self.home_reference is not None
                    and (not self.hardware or self.calibrated),
                    "physical_estop": False,
                },
                "processes": {
                    "reader": self.reader is not None and self.reader.poll() is None,
                    "motion": self.motion_process is not None
                    and self.motion_process.poll() is None,
                    "gravity": self.gravity_process is not None
                    and self.gravity_process.poll() is None,
                },
            }

    def _require_ready(self) -> None:
        if not self.connected:
            raise ActionError("控制器未连接", 409)
        if self.estop:
            raise ActionError("急停状态不允许运动", 409)

    def start_reader(self) -> None:
        if not self.hardware:
            return
        with self.lock:
            if self.reader is not None and self.reader.poll() is None:
                return
            command = [
                self.reader_path,
                "--device",
                "/dev/ttyUSB0",
                "--ids",
                ",".join(str(joint["id"]) for joint in self.joints),
                "--mode",
                "brake",
                "--rate",
                "100",
                "--acknowledge-state-change",
            ]
            try:
                self.reader = self._popen(
                    command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1
                )
            except OSError as exc:
                self.reader = None
                raise ActionError(f"反馈读取器启动失败: {exc}", 503) from exc
            self.reader_thread = threading.Thread(target=self._read_reader, daemon=True)
            self.reader_thread.start()

    def _read_reader(self) -> None:
        reader = self.reader
        if reader is None or reader.stdout is None:
            return
        for line in reader.stdout:
            try:
                raw = json.loads(line)
            except (json.JSONDecodeError, TypeError):
                continue
            if not isinstance(raw, dict) or raw.get("type") != "sample":
                continue
            motors = raw.get("motors")
            if not isinstance(motors, list) or len(motors) != self.dof:
                continue
            if any(
                not isinstance(item, dict) or type(item.get("id")) is not int for item in motors
            ):
                continue
            by_id = {item["id"]: item for item in motors}
            if set(by_id) != {joint["id"] for joint in self.joints}:
                continue
            updates = []
            try:
                for index, joint in enumerate(self.joints):
                    item = by_id[joint["id"]]
                    direction = self.mapping["direction"][index] if self.calibrated else 1
                    offset = self.mapping["zero_offset_rad"][index] if self.calibrated else 0
                    source_position = float(item[self.mapping.get("angle_field", "q_output_rad")])
                    calibration = self.mapping.get("calibration", {})
                    if self.calibrated and "reference_joint_rad" in calibration:
                        angle = calibration["reference_joint_rad"][index] + direction * (
                            source_position - calibration["source_at_reference_rad"][index]
                        )
                    else:
                        angle = direction * (source_position - offset)
                    values = {
                        "angle": angle,
                        "velocity": direction
                        * float(item[self.mapping.get("velocity_field", "dq_output_rad_s")]),
                        "torque": direction
                        * float(item[self.mapping.get("torque_field", "tau_ideal_output_nm")]),
                        "temperature": float(item["temperature_c"]),
                        "error": int(item.get("error", 0)),
                    }
                    if not all(math.isfinite(value) for value in values.values()):
                        raise ValueError("non-finite telemetry")
                    updates.append(values)
            except (KeyError, TypeError, ValueError, OverflowError):
                continue
            with self.lock:
                for joint, values in zip(self.joints, updates, strict=True):
                    joint.update(values)
                self.connected = True
                if self.lifecycle in {"connecting", "disconnected"}:
                    self.lifecycle = "connected_read_only"
                    self._set_action(
                        "feedback_online", f"{self.dof} 轴反馈在线（读取器使用 BRAKE 轮询）"
                    )

    def stop_reader(self) -> None:
        with self.lock:
            process, self.reader = self.reader, None
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()

    def _start_process(self, command: list[str], kind: str) -> subprocess.Popen[str]:
        try:
            return self._popen(
                command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, text=True
            )
        except OSError as exc:
            raise ActionError(f"{kind}进程启动失败: {exc}", 503) from exc

    def connect(self) -> dict[str, Any]:
        with self.lock:
            if not self.hardware:
                self.connected, self.lifecycle = True, "simulation_ready"
                self._set_action("connect", "仿真控制器已连接")
            else:
                self.lifecycle = "connecting"
                self._set_action("connect", f"正在等待 {self.dof} 轴反馈；尚未使能")
        if self.hardware:
            self.start_reader()
        return self.snapshot()

    def disconnect(self) -> dict[str, Any]:
        self.stop_reader()
        with self.lock:
            for name in ("motion_process", "gravity_process"):
                process = getattr(self, name)
                if process is not None and process.poll() is None:
                    process.terminate()
                setattr(self, name, None)
            self.enabled = self.gravity = False
            self.connected = not self.hardware
            self.lifecycle = "disconnected" if self.hardware else "simulation_ready"
            self._set_action("disconnect", "控制器已断开" if self.hardware else "仿真连接已重置")
        return self.snapshot()

    def estop_action(self) -> dict[str, Any]:
        with self.lock:
            self.estop, self.enabled, self.gravity = True, False, False
            self.lifecycle = "fault" if self.hardware else "simulation_estop"
            for name in ("motion_process", "gravity_process"):
                process = getattr(self, name)
                if process is not None and process.poll() is None:
                    process.terminate()
                setattr(self, name, None)
            msg = (
                "急停锁存；硬件 BRAKE 适配器未接入，请使用物理急停"
                if self.hardware
                else "急停已触发，仿真动作停止"
            )
            self._set_action("estop", msg)
        return self.snapshot()

    def clear_estop_action(self) -> dict[str, Any]:
        with self.lock:
            self.estop = False
            self.lifecycle = (
                "connected_read_only"
                if self.hardware and self.connected
                else "simulation_ready"
                if not self.hardware
                else "disconnected"
            )
            self._set_action("clear_estop", "急停已解除，请重新使能")
        return self.snapshot()

    def set_enabled(self, requested: bool) -> dict[str, Any]:
        with self.lock:
            if requested:
                self._require_ready()
                if self.hardware:
                    raise ActionError("硬件使能适配器未接入；当前仅支持反馈读取", 501)
                self.enabled, self.lifecycle = True, "holding"
                self._set_action("enable", "仿真机械臂已使能")
            else:
                self.enabled = self.gravity = False
                self.lifecycle = "connected_read_only" if self.hardware else "simulation_ready"
                self._set_action("disable", "机械臂已掉使能")
        return self.snapshot()

    def set_gravity(self, requested: bool) -> dict[str, Any]:
        with self.lock:
            if requested:
                self._require_ready()
                if not self.enabled:
                    raise ActionError("请先使能机械臂", 409)
                if self.hardware:
                    raise ActionError("硬件重力补偿适配器未接入；状态保持关闭", 501)
                self.gravity, self.lifecycle = True, "gravity_compensating"
                self._set_action("gravity_on", "仿真 100% 重力补偿已开启")
            else:
                self.gravity = False
                self.lifecycle = "holding" if self.enabled else self.lifecycle
                self._set_action("gravity_off", "重力补偿已关闭")
        return self.snapshot()

    def movej(self, joints: Any, speed: Any = 0.25, acceleration: Any = 0.5) -> dict[str, Any]:
        if not isinstance(joints, list) or len(joints) != self.dof:
            raise ActionError(f"joints must contain {self.dof} radians", 400)
        if any(type(value) not in (int, float) for value in [*joints, speed, acceleration]):
            raise ActionError("joints, speed and acceleration must be numbers", 400)
        try:
            values = [float(x) for x in joints]
            speed_value, accel_value = float(speed), float(acceleration)
        except (TypeError, ValueError) as exc:
            raise ActionError("joints, speed and acceleration must be numeric", 400) from exc
        if (
            not all(math.isfinite(x) for x in values)
            or not math.isfinite(speed_value)
            or not math.isfinite(accel_value)
        ):
            raise ActionError("joints, speed and acceleration must be finite", 400)
        if speed_value <= 0 or accel_value <= 0:
            raise ActionError("speed and acceleration must be positive", 400)
        if any(
            value < lower or value > upper
            for value, (lower, upper) in zip(
                values, [(joint["min"], joint["max"]) for joint in self.joints], strict=True
            )
        ):
            raise ActionError("joint angle outside configured soft limit", 400)
        with self.lock:
            self._require_ready()
            if not self.enabled:
                raise ActionError("请先使能机械臂", 409)
            if self.hardware:
                raise ActionError("MOVEJ 适配器未接入；请使用离线规划器", 501)
            for i, value in enumerate(values):
                self.joints[i]["angle"], self.joints[i]["velocity"] = value, 0.0
            self.lifecycle = "holding"
            self._set_action(
                "movej", f"仿真 MOVEJ 已完成 (speed={speed_value:g}, acceleration={accel_value:g})"
            )
        return self.snapshot()

    def return_home(self, trajectory: str, confirmed: bool) -> dict[str, Any]:
        if not trajectory or not Path(trajectory).is_file():
            raise ActionError("trajectory_path must be an existing calibration_home.csv", 400)
        if not confirmed:
            raise ActionError("confirm_collision_checked_plan is required", 400)
        final = read_trajectory(trajectory, self.joint_names)
        if self.home_reference is None or (self.hardware and not self.calibrated):
            raise ActionError("当前机械臂尚未完成有效标定，回标定位不可用", 409)
        if any(
            abs(value - expected) > 1e-6
            for value, expected in zip(final, self.home_reference, strict=True)
        ):
            raise ActionError(
                "trajectory endpoint does not match the validated calibration pose", 400
            )
        with self.lock:
            self._require_ready()
            if not self.enabled:
                raise ActionError("请先使能机械臂", 409)
            command = [
                self.return_home_path,
                "--trajectory",
                trajectory,
                "--enable-foc",
                "--acknowledge-supported-arm",
                "--acknowledge-estop-ready",
                "--confirm-same-motor-power-cycle",
                "--confirm-collision-checked-plan",
            ]
            if self.hardware:
                if self.motion_process is not None and self.motion_process.poll() is None:
                    raise ActionError("回标定位已经在执行", 409)
                # The readonly reader owns the same bus lock and sends BRAKE on
                # every poll. Stop it before handing the bus to the trajectory
                # controller; otherwise both processes would race the motors.
                self.stop_reader()
                self.motion_process = self._start_process(command, "回标定位")
                self.lifecycle = "moving_to_calibration"
                self._set_action("return_home_started", "回标定位已启动；等待进程完成")
            else:
                for i, value in enumerate(final):
                    self.joints[i]["angle"], self.joints[i]["velocity"] = value, 0.0
                self.lifecycle = "holding"
                self._set_action("return_home", "仿真已回到桌面支撑标定位")
        return self.snapshot()


def validate_program(program: Any, joints: list[dict[str, Any]]) -> list[str]:
    """Validate versioned online-program JSON without executing it."""
    errors: list[str] = []
    if not isinstance(program, dict):
        return ["program must be an object"]
    if program.get("version") != 1:
        errors.append("version must be 1")
    nodes = program.get("nodes")
    if not isinstance(nodes, list) or not nodes:
        return errors + ["nodes must be a non-empty array"]
    allowed = {"start", "end", "movej", "wait"}
    expected_names = [joint["joint_name"] for joint in joints]
    if program.get("joint_names") != expected_names:
        errors.append("joint_names must exactly match the active URDF chain")
    for index, node in enumerate(nodes):
        if not isinstance(node, dict):
            errors.append(f"nodes[{index}] must be an object")
            continue
        kind = node.get("type")
        if kind not in allowed:
            errors.append(f"nodes[{index}].type is unsupported")
        if kind == "movej":
            values = node.get("joints")
            valid_joints = (
                isinstance(values, list)
                and len(values) == len(joints)
                and all(type(x) in (int, float) and math.isfinite(float(x)) for x in values)
            )
            if not valid_joints:
                errors.append(f"nodes[{index}].joints must contain {len(joints)} finite radians")
            elif any(
                value < joint["min"] or value > joint["max"]
                for value, joint in zip(values, joints, strict=True)
            ):
                errors.append(f"nodes[{index}].joints exceed active soft limits")
        if kind == "wait":
            try:
                duration = float(node.get("duration_s"))
            except (TypeError, ValueError):
                duration = -1
            if not math.isfinite(duration) or duration < 0 or duration > 3600:
                errors.append(f"nodes[{index}].duration_s must be in [0,3600]")
    if not isinstance(nodes[0], dict) or nodes[0].get("type") != "start":
        errors.append("first node must be start")
    if not isinstance(nodes[-1], dict) or nodes[-1].get("type") != "end":
        errors.append("last node must be end")
    return errors


class Handler(BaseHTTPRequestHandler):
    server_version = "QarmControl/0.2"

    def log_message(self, _format: str, *_args: object) -> None:
        return

    @property
    def state(self) -> ControlState:
        return self.server.control_state  # type: ignore[attr-defined]

    def _send(self, status: int, payload: object) -> None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(data)

    def _body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if not length:
            return {}
        value = json.loads(self.rfile.read(length))
        if not isinstance(value, dict):
            raise ValueError("request body must be an object")
        return value

    def do_OPTIONS(self) -> None:  # noqa: N802
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self) -> None:  # noqa: N802
        if self.path == "/api/health":
            self._send(200, {"ok": True, "service": "qarm-control", "mode": self.state.mode})
        elif self.path in {"/api/status", "/api/v1/state"}:
            self._send(200, self.state.snapshot())
        elif self.path == "/api/joints":
            self._send(200, {"joints": self.state.snapshot()["joints"]})
        elif self.path in {"/api/events", "/api/v1/events"}:
            self._send(200, {"events": list(self.state.events)})
        elif self.path in {"/api/config", "/api/v1/config"}:
            self._send(
                200,
                {
                    "path": str(self.state.config_path),
                    "urdf": str(self.state.urdf_path),
                    "dof": self.state.dof,
                    "joint_names": self.state.joint_names,
                    "motor_ids_by_joint": [joint["id"] for joint in self.state.joints],
                    "calibrated": self.state.calibrated,
                    "hardware": self.state.hardware,
                    "return_home": self.state.return_home_path,
                    "joint_limits_rad": [
                        (joint["min"], joint["max"]) for joint in self.state.joints
                    ],
                    "calibration_pose_rad": self.state.home_reference,
                },
            )
        elif not self.path.startswith("/api/"):
            relative = self.path.split("?", 1)[0].lstrip("/") or "index.html"
            root = DIST.resolve()
            candidate = (DIST / relative).resolve()
            if DIST.exists() and root in candidate.parents and candidate.is_file():
                content_type = {
                    ".html": "text/html; charset=utf-8",
                    ".js": "application/javascript",
                    ".css": "text/css",
                }.get(candidate.suffix, "application/octet-stream")
                data = candidate.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(data)
            else:
                index = (DIST / "index.html").resolve()
                if DIST.exists() and index.is_file():
                    data = index.read_bytes()
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                else:
                    self._send(
                        404, {"error": "platform bundle not found; build platform/dist first"}
                    )
        else:
            self._send(404, {"error": "not found"})

    def do_POST(self) -> None:  # noqa: N802
        try:
            body = self._body()
            if self.path == "/api/connect":
                return self._send(200, self.state.connect())
            if self.path == "/api/disconnect":
                return self._send(200, self.state.disconnect())
            if self.path == "/api/estop":
                return self._send(200, self.state.estop_action())
            if self.path == "/api/clear-estop":
                return self._send(200, self.state.clear_estop_action())
            if self.path == "/api/enable":
                return self._send(200, self.state.set_enabled(bool(body.get("enabled", True))))
            if self.path == "/api/gravity":
                return self._send(200, self.state.set_gravity(bool(body.get("enabled", True))))
            if self.path == "/api/movej":
                return self._send(
                    200,
                    self.state.movej(
                        body.get("joints"), body.get("speed", 0.25), body.get("acceleration", 0.5)
                    ),
                )
            if self.path == "/api/return-home":
                return self._send(
                    202 if self.state.hardware else 200,
                    self.state.return_home(
                        str(body.get("trajectory_path", "")),
                        bool(body.get("confirm_collision_checked_plan")),
                    ),
                )
            if self.path in {"/api/stop", "/api/v1/stop"}:
                return self._send(200, self.state.estop_action())
            if self.path in {"/api/program/validate", "/api/v1/program/validate"}:
                errors = validate_program(body.get("program", body), self.state.joints)
                return self._send(
                    200 if not errors else 422, {"valid": not errors, "errors": errors}
                )
            self._send(404, {"error": "not found"})
        except ActionError as error:
            self._send(error.status, {"error": str(error), "status": self.state.snapshot()})
        except (ValueError, json.JSONDecodeError, OSError) as error:
            self._send(400, {"error": str(error)})


# Backwards-compatible process-wide state for small scripts that imported the
# original module-level ``STATE``. ``create_server`` still creates isolated
# state by default, which keeps tests and embedded instances independent.
STATE = ControlState()


def create_server(
    state: ControlState | None = None, host: str = "0.0.0.0", port: int = PORT
) -> ThreadingHTTPServer:
    httpd = ThreadingHTTPServer((host, port), Handler)
    httpd.control_state = state or ControlState()  # type: ignore[attr-defined]
    return httpd


def main() -> None:
    httpd = create_server()
    state: ControlState = httpd.control_state  # type: ignore[attr-defined]
    print(f"Qarm control server listening on :{PORT} mode={state.mode}", flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        state.disconnect()
        httpd.server_close()


if __name__ == "__main__":
    main()
