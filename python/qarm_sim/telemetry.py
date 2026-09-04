from __future__ import annotations

import json
import queue
import subprocess
import threading
import time
from collections.abc import Iterable, Iterator
from contextlib import suppress
from dataclasses import dataclass, replace
from pathlib import Path
from typing import IO, Any

import numpy as np
from numpy.typing import NDArray

from qarm_sim.config import JointMap


class TelemetryError(RuntimeError):
    """Raised for malformed or unavailable motor telemetry."""


@dataclass(frozen=True)
class MotorFeedback:
    motor_id: int
    correct: bool
    q_sdk_rad: float
    dq_sdk_rad_s: float
    q_output_rad: float
    dq_output_rad_s: float
    tau_sdk_nm: float
    tau_ideal_output_nm: float
    temperature_c: int
    error: int

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> MotorFeedback:
        return cls(
            motor_id=int(raw["id"]),
            correct=bool(raw["correct"]),
            q_sdk_rad=float(raw["q_sdk_rad"]),
            dq_sdk_rad_s=float(raw["dq_sdk_rad_s"]),
            q_output_rad=float(raw["q_output_rad"]),
            dq_output_rad_s=float(raw["dq_output_rad_s"]),
            tau_sdk_nm=float(raw["tau_sdk_nm"]),
            tau_ideal_output_nm=float(raw["tau_ideal_output_nm"]),
            temperature_c=int(raw["temperature_c"]),
            error=int(raw["error"]),
        )

    def field(self, name: str) -> float:
        try:
            value = getattr(self, name)
        except AttributeError as error:
            raise TelemetryError(f"motor sample has no field {name!r}") from error
        if not isinstance(value, (float, int)):
            raise TelemetryError(f"motor field {name!r} is not numeric")
        return float(value)


@dataclass(frozen=True)
class TelemetrySample:
    monotonic_ns: int
    host_receive_monotonic_ns: int | None
    sequence: int
    motors: tuple[MotorFeedback, ...]

    @classmethod
    def from_line(cls, line: str) -> TelemetrySample | None:
        try:
            raw = json.loads(line)
        except json.JSONDecodeError as error:
            raise TelemetryError(f"invalid telemetry JSON: {error}") from error
        if not isinstance(raw, dict):
            raise TelemetryError("telemetry line must be a JSON object")
        if raw.get("type") != "sample":
            return None
        motors = tuple(MotorFeedback.from_dict(item) for item in raw["motors"])
        if len({motor.motor_id for motor in motors}) != len(motors):
            raise TelemetryError("sample contains duplicate motor IDs")
        return cls(
            monotonic_ns=int(raw["monotonic_ns"]),
            host_receive_monotonic_ns=(
                int(raw["host_receive_monotonic_ns"])
                if "host_receive_monotonic_ns" in raw
                else None
            ),
            sequence=int(raw["sequence"]),
            motors=motors,
        )


@dataclass(frozen=True)
class JointState:
    position: NDArray[np.float64]
    velocity: NDArray[np.float64]
    torque: NDArray[np.float64]
    temperature_c: NDArray[np.int64]
    motor_error: NDArray[np.int64]


def map_joint_state(sample: TelemetrySample, mapping: JointMap) -> JointState:
    by_id = {motor.motor_id: motor for motor in sample.motors}
    missing = [
        int(motor_id)
        for motor_id in mapping.motor_ids_by_joint
        if int(motor_id) not in by_id
    ]
    if missing:
        raise TelemetryError(f"sample is missing motor IDs {missing}")
    ordered = [by_id[int(motor_id)] for motor_id in mapping.motor_ids_by_joint]
    invalid = [motor.motor_id for motor in ordered if not motor.correct]
    if invalid:
        raise TelemetryError(f"invalid feedback CRC/status for motor IDs {invalid}")
    source_position = np.array(
        [motor.field(mapping.angle_field) for motor in ordered],
        dtype=np.float64,
    )
    source_velocity = np.array(
        [motor.field(mapping.velocity_field) for motor in ordered],
        dtype=np.float64,
    )
    if (
        mapping.zero_calibrated
        and mapping.calibration_reference_joint_rad is not None
        and mapping.source_at_reference_rad is not None
    ):
        position = mapping.calibration_reference_joint_rad + (
            mapping.direction
            * (source_position - mapping.source_at_reference_rad)
        )
    else:
        position = mapping.direction * (
            source_position - mapping.zero_offset_rad
        )
    velocity = mapping.direction * source_velocity
    torque = mapping.direction * np.array(
        [motor.field(mapping.torque_field) for motor in ordered],
        dtype=np.float64,
    )
    if not np.isfinite(position).all() or not np.isfinite(velocity).all():
        raise TelemetryError("mapped joint state contains non-finite values")
    return JointState(
        position=position,
        velocity=velocity,
        torque=torque,
        temperature_c=np.array(
            [motor.temperature_c for motor in ordered], dtype=np.int64
        ),
        motor_error=np.array(
            [motor.error for motor in ordered], dtype=np.int64
        ),
    )


def iter_samples(stream: Iterable[str]) -> Iterator[TelemetrySample]:
    for line in stream:
        line = line.strip()
        if not line:
            continue
        sample = TelemetrySample.from_line(line)
        if sample is not None:
            yield sample


class SubprocessTelemetry:
    """Read newline-delimited telemetry without blocking the viewer thread."""

    def __init__(self, command: list[str], *, record_path: Path | None = None):
        self.command = command
        self.record_path = record_path
        self.process: subprocess.Popen[str] | None = None
        self._queue: queue.Queue[TelemetrySample | BaseException] = queue.Queue(
            maxsize=4
        )
        self._thread: threading.Thread | None = None
        self._record: IO[str] | None = None

    def start(self) -> None:
        if self.process is not None:
            raise RuntimeError("telemetry process has already started")
        if self.record_path is not None:
            self.record_path.parent.mkdir(parents=True, exist_ok=True)
            self._record = self.record_path.open("a", encoding="utf-8")
        self.process = subprocess.Popen(
            self.command,
            stdout=subprocess.PIPE,
            stderr=None,
            text=True,
            bufsize=1,
        )
        if self.process.stdout is None:
            raise TelemetryError("telemetry subprocess has no stdout")
        self._thread = threading.Thread(
            target=self._read_loop, args=(self.process.stdout,), daemon=True
        )
        self._thread.start()

    def _offer(self, item: TelemetrySample | BaseException) -> None:
        while True:
            try:
                self._queue.put_nowait(item)
                return
            except queue.Full:
                with suppress(queue.Empty):
                    self._queue.get_nowait()

    def _read_loop(self, stream: IO[str]) -> None:
        try:
            for line in stream:
                received_ns = time.monotonic_ns()
                stripped = line.strip()
                if not stripped.startswith("{"):
                    # Some SDK builds print timeout diagnostics to stdout.
                    if self._record is not None:
                        self._record.write(
                            json.dumps(
                                {
                                    "type": "reader_log",
                                    "host_receive_monotonic_ns": received_ns,
                                    "message": stripped,
                                },
                                separators=(",", ":"),
                            )
                            + "\n"
                        )
                        self._record.flush()
                    continue
                sample = TelemetrySample.from_line(stripped)
                if self._record is not None:
                    raw = json.loads(stripped)
                    raw["host_receive_monotonic_ns"] = received_ns
                    self._record.write(
                        json.dumps(raw, separators=(",", ":")) + "\n"
                    )
                    self._record.flush()
                if sample is not None:
                    self._offer(
                        replace(
                            sample, host_receive_monotonic_ns=received_ns
                        )
                    )
        except BaseException as error:
            self._offer(error)
        else:
            self._offer(TelemetryError("telemetry stream ended"))

    def latest(self, timeout: float | None = None) -> TelemetrySample:
        try:
            item = self._queue.get(timeout=timeout)
        except queue.Empty as error:
            if self.process is not None and self.process.poll() is not None:
                raise TelemetryError(
                    f"telemetry process exited with {self.process.returncode}"
                ) from error
            raise TelemetryError("timed out waiting for motor telemetry") from error
        while True:
            try:
                item = self._queue.get_nowait()
            except queue.Empty:
                break
        if isinstance(item, BaseException):
            raise TelemetryError(str(item)) from item
        return item

    def close(self) -> None:
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        if self._thread is not None:
            self._thread.join(timeout=2)
        if self._record is not None:
            self._record.close()

    def __enter__(self) -> SubprocessTelemetry:
        self.start()
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()
