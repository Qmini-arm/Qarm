"""Small, guarded Python binding for the Unitree GO-M8010-6 serial bus.

This module is deliberately independent of the control state machine.  It only
translates validated rotor-side frames to the vendor Python extension and
normalizes feedback.  Importing this module never opens a serial device.
"""

from __future__ import annotations

import fcntl
import importlib
import math
import os
import stat
import sys
import threading
from collections.abc import Callable
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType
from typing import Any


class UnitreeBusError(RuntimeError):
    """A transport or feedback validation failure."""


@dataclass(frozen=True, slots=True)
class MotorCommand:
    """Complete rotor-side command sent for one M8010 motor."""

    motor_id: int
    q_rad: float = 0.0
    dq_rad_s: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    tau_nm: float = 0.0
    mode: str = "BRAKE"

    def __post_init__(self) -> None:
        if type(self.motor_id) is not int or not 0 <= self.motor_id <= 14:
            raise ValueError("motor_id must be an integer in [0, 14]")
        if self.mode not in {"BRAKE", "FOC", "CALIBRATE"}:
            raise ValueError("mode must be BRAKE, FOC, or CALIBRATE")
        values = (self.q_rad, self.dq_rad_s, self.kp, self.kd, self.tau_nm)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("motor command values must be finite")
        if self.kp < 0.0 or self.kd < 0.0:
            raise ValueError("motor command gains must be non-negative")


@dataclass(frozen=True, slots=True)
class MotorFeedback:
    motor_id: int
    q_rad: float
    dq_rad_s: float
    tau_nm: float
    temperature_c: float
    error: int
    mode: int


@dataclass(frozen=True, slots=True)
class UnitreeBusConfig:
    """Validated device and motor mapping, before opening ``SerialPort``."""

    device: str
    motor_ids: tuple[int, ...] = (0, 1, 2, 3)
    gear_ratio: float = 6.33
    sdk_path: str | None = None
    lock_path: str = "/tmp/qarm_m8010_bus.lock"
    timeout_us: int = 20_000

    def __post_init__(self) -> None:
        device = str(self.device).strip()
        if not device:
            raise ValueError("device must not be empty")
        object.__setattr__(self, "device", device)
        ids = tuple(int(value) for value in self.motor_ids)
        if not ids or len(set(ids)) != len(ids) or any(value < 0 or value > 14 for value in ids):
            raise ValueError("motor_ids must be unique IDs in [0, 14]")
        object.__setattr__(self, "motor_ids", ids)
        if not math.isfinite(float(self.gear_ratio)) or self.gear_ratio <= 0.0:
            raise ValueError("gear_ratio must be positive and finite")
        if type(self.timeout_us) is not int or not 100 <= self.timeout_us <= 1_000_000:
            raise ValueError("timeout_us must be in [100, 1000000]")
        lock = str(self.lock_path).strip()
        if not lock:
            raise ValueError("lock_path must not be empty")
        object.__setattr__(self, "lock_path", lock)

    def validate_device(self) -> Path:
        """Validate a character device and permissions without opening it."""
        path = Path(self.device)
        try:
            info = path.stat()
        except OSError as exc:
            raise UnitreeBusError(f"serial device is unavailable: {path}: {exc}") from exc
        if not stat.S_ISCHR(info.st_mode):
            raise UnitreeBusError(f"serial path is not a character device: {path}")
        if not os.access(path, os.R_OK | os.W_OK):
            raise UnitreeBusError(f"serial device is not readable and writable: {path}")
        return path


def _load_sdk(path: str | None, module: ModuleType | None) -> ModuleType:
    if module is not None:
        return module
    sdk_dir = (
        Path(path).expanduser().resolve()
        if path
        else (Path(__file__).resolve().parents[3] / "unitree_actuator_sdk" / "lib")
    )
    if sdk_dir:
        if not sdk_dir.is_dir():
            raise UnitreeBusError(f"Unitree Python SDK directory does not exist: {sdk_dir}")
        if str(sdk_dir) not in sys.path:
            sys.path.insert(0, str(sdk_dir))
    try:
        return importlib.import_module("unitree_actuator_sdk")
    except (ImportError, OSError) as exc:
        raise UnitreeBusError(
            "cannot import unitree_actuator_sdk; set sdk_path to the directory "
            "containing the aarch64 Python extension"
        ) from exc


class _BusLock:
    def __init__(self, path: str) -> None:
        descriptor = os.open(path, os.O_CREAT | os.O_RDWR, 0o600)
        self._file = os.fdopen(descriptor, "a+b", buffering=0)
        try:
            fcntl.flock(self._file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            self._file.close()
            raise UnitreeBusError(f"another process owns the motor bus lock: {path}") from exc

    def close(self) -> None:
        try:
            fcntl.flock(self._file.fileno(), fcntl.LOCK_UN)
        finally:
            self._file.close()


class UnitreeM8010Bus:
    """Validated, exclusive SDK transport for GO-M8010-6 motors.

    ``serial_factory`` is injectable for tests.  Production code leaves it
    unset, which constructs the vendor ``SerialPort`` only after all checks.
    """

    def __init__(
        self,
        config: UnitreeBusConfig,
        *,
        sdk_module: ModuleType | None = None,
        serial_factory: Callable[[str], Any] | None = None,
    ) -> None:
        config.validate_device()
        sdk = _load_sdk(config.sdk_path, sdk_module)
        for name in (
            "SerialPort",
            "MotorCmd",
            "MotorData",
            "MotorType",
            "MotorMode",
            "queryMotorMode",
        ):
            if not hasattr(sdk, name):
                raise UnitreeBusError(f"Unitree SDK is missing required symbol: {name}")
        self.config = config
        self.sdk = sdk
        self._lock = _BusLock(config.lock_path)
        try:
            self._serial = (serial_factory or sdk.SerialPort)(config.device)
        except Exception:
            self._lock.close()
            raise
        self._closed = False
        self._io_lock = threading.Lock()
        self._type = sdk.MotorType.GO_M8010_6
        self._modes = {
            name: int(sdk.queryMotorMode(self._type, getattr(sdk.MotorMode, name)))
            for name in ("BRAKE", "FOC", "CALIBRATE")
        }

    @property
    def brake_mode(self) -> int:
        return self._modes["BRAKE"]

    @property
    def foc_mode(self) -> int:
        return self._modes["FOC"]

    def exchange(self, command: MotorCommand) -> MotorFeedback:
        if self._closed:
            raise UnitreeBusError("motor bus is closed")
        sdk_command = self.sdk.MotorCmd()
        sdk_state = self.sdk.MotorData()
        sdk_command.motorType = self._type
        sdk_state.motorType = self._type
        sdk_command.id = command.motor_id
        sdk_command.mode = self._modes[command.mode]
        sdk_command.q = command.q_rad
        sdk_command.dq = command.dq_rad_s
        sdk_command.kp = command.kp
        sdk_command.kd = command.kd
        sdk_command.tau = command.tau_nm
        sdk_state.correct = False
        try:
            with self._io_lock:
                ok = bool(self._serial.sendRecv(sdk_command, sdk_state))
        except Exception as exc:
            raise UnitreeBusError(f"motor {command.motor_id} exchange failed: {exc}") from exc
        returned = int(sdk_state.motor_id)
        if not ok or not bool(sdk_state.correct):
            raise UnitreeBusError(f"motor {command.motor_id} timeout or CRC failure")
        if returned != command.motor_id:
            raise UnitreeBusError(
                f"motor reply ID mismatch: expected {command.motor_id}, got {returned}"
            )
        values = (sdk_state.q, sdk_state.dq, sdk_state.tau, sdk_state.temp)
        if not all(math.isfinite(float(value)) for value in values):
            raise UnitreeBusError(f"motor {command.motor_id} returned non-finite feedback")
        return MotorFeedback(
            motor_id=returned,
            q_rad=float(sdk_state.q),
            dq_rad_s=float(sdk_state.dq),
            tau_nm=float(sdk_state.tau),
            temperature_c=float(sdk_state.temp),
            error=int(sdk_state.merror),
            mode=int(sdk_state.mode),
        )

    def brake(self, motor_id: int) -> MotorFeedback:
        return self.exchange(MotorCommand(motor_id=motor_id, mode="BRAKE"))

    def brake_all(self) -> tuple[MotorFeedback, ...]:
        feedback: list[MotorFeedback] = []
        for motor_id in self.config.motor_ids:
            feedback.append(self.brake(motor_id))
        return tuple(feedback)

    def close(self) -> None:
        if self._closed:
            return
        with suppress(Exception):
            self.brake_all()
        self._closed = True
        close = getattr(self._serial, "close", None)
        if callable(close):
            close()
        self._lock.close()

    def __enter__(self) -> UnitreeM8010Bus:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
