#!/usr/bin/env python3
"""Qarm control-plane HTTP service.

The browser talks to this process; it never opens ``/dev/ttyUSB0`` itself. The
default is deterministic simulation. Hardware mode is capability-gated: until
a real controller adapter is installed, requests that would energise or command
motors return ``501`` and the state is not changed.
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import threading
import time
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
HOME_REFERENCE = [0.0, 1.748017811, 0.154806471, -0.020052336, 0.0, -1.57]
JOINT_LIMITS = [
    (-2.967, 2.967),
    (-1.57, 1.57),
    (-2.007, 2.007),
    (-2.0, 2.0),
    (-2.007, 2.007),
    (-1.5, 1.5),
]
DIST = Path(os.environ.get("QARM_PLATFORM_DIST", str(ROOT / "platform" / "dist")))


def _default_joints() -> list[dict[str, Any]]:
    values = [-0.473, 0.883, -1.662, -1.268, -0.020, -0.495]
    return [
        {
            "name": f"J{i + 1}",
            "id": i,
            "angle": values[i],
            "velocity": 0.0,
            "torque": 0.0,
            "temperature": 30.0,
            "error": 0,
            "min": JOINT_LIMITS[i][0],
            "max": JOINT_LIMITS[i][1],
        }
        for i in range(6)
    ]


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
        self.joints = _default_joints()
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
                "last_action": self.last_action,
                "notice": self.notice,
                "hardware_io_enabled": self.hardware,
                "reader": self.reader_path if self.hardware else None,
                "capabilities": {
                    "feedback": self.hardware,
                    "enable": not self.hardware,
                    "gravity": not self.hardware,
                    "movej": not self.hardware,
                    "return_home": True,
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
                "0,1,2,3,4,5",
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
            if raw.get("type") != "sample":
                continue
            by_id = {
                int(item["id"]): item
                for item in raw.get("motors", [])
                if isinstance(item, dict) and "id" in item
            }
            if any(i not in by_id for i in range(6)):
                continue
            with self.lock:
                for joint in self.joints:
                    item = by_id[joint["id"]]
                    joint["angle"] = float(item.get("q_output_rad", joint["angle"]))
                    joint["velocity"] = float(item.get("dq_output_rad_s", joint["velocity"]))
                    joint["torque"] = float(item.get("tau_ideal_output_nm", joint["torque"]))
                    joint["temperature"] = float(item.get("temperature_c", joint["temperature"]))
                    joint["error"] = int(item.get("error", 0))
                self.connected = True
                if self.lifecycle in {"connecting", "disconnected"}:
                    self.lifecycle = "connected_read_only"
                    self._set_action("feedback_online", "六轴反馈在线（读取器使用 BRAKE 轮询）")

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
                self._set_action("connect", "正在等待六轴反馈；尚未使能")
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
        if not isinstance(joints, list) or len(joints) != 6:
            raise ActionError("joints must contain six radians", 400)
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
            for value, (lower, upper) in zip(values, JOINT_LIMITS, strict=True)
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
                for i, value in enumerate(HOME_REFERENCE):
                    self.joints[i]["angle"], self.joints[i]["velocity"] = value, 0.0
                self.lifecycle = "holding"
                self._set_action("return_home", "仿真已回到桌面支撑标定位")
        return self.snapshot()


def validate_program(program: Any) -> list[str]:
    """Validate versioned online-program JSON without executing it."""
    errors: list[str] = []
    if not isinstance(program, dict):
        return ["program must be an object"]
    if program.get("version") != 1:
        errors.append("version must be 1")
    nodes = program.get("nodes")
    if not isinstance(nodes, list) or not nodes:
        return errors + ["nodes must be a non-empty array"]
    allowed = {"start", "end", "movej", "movel", "wait", "set", "if", "loop", "popup"}
    for index, node in enumerate(nodes):
        if not isinstance(node, dict):
            errors.append(f"nodes[{index}] must be an object")
            continue
        kind = node.get("type")
        if kind not in allowed:
            errors.append(f"nodes[{index}].type is unsupported")
        if kind == "movej":
            joints = node.get("joints")
            valid_joints = (
                isinstance(joints, list)
                and len(joints) == 6
                and all(isinstance(x, (int, float)) and math.isfinite(float(x)) for x in joints)
            )
            if not valid_joints:
                errors.append(f"nodes[{index}].joints must contain six finite radians")
        if kind == "wait":
            try:
                duration = float(node.get("duration_s"))
            except (TypeError, ValueError):
                duration = -1
            if not math.isfinite(duration) or duration < 0 or duration > 3600:
                errors.append(f"nodes[{index}].duration_s must be in [0,3600]")
    if nodes[0].get("type") != "start":
        errors.append("first node must be start")
    if nodes[-1].get("type") != "end":
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
                    "path": str(CONFIG),
                    "hardware": self.state.hardware,
                    "return_home": self.state.return_home_path,
                    "joint_limits_rad": JOINT_LIMITS,
                    "calibration_pose_rad": HOME_REFERENCE,
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
                errors = validate_program(body.get("program", body))
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
