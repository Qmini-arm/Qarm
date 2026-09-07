"""Real GO-M8010-6 backend using the Python Unitree SDK.

The state machine remains the same protocol as :class:`FakeArmBackend`; only
the serial exchange is owned by a private worker thread.  Opening the device
is deferred until ``connection.connect`` is explicitly accepted.
"""

from __future__ import annotations

import copy
import math
import threading
import time
from collections.abc import Mapping, Sequence
from contextlib import suppress
from dataclasses import replace
from datetime import datetime, timezone
from typing import Any

from .backend import CommandRejected, FakeArmBackend
from .schemas import ControllerState, stable_id
from .unitree_bus import (
    MotorCommand,
    MotorFeedback,
    UnitreeBusConfig,
    UnitreeM8010Bus,
)


class HardwareArmBackend(FakeArmBackend):
    def __init__(
        self,
        *,
        device: str,
        motor_ids: Sequence[int],
        sdk_path: str | None = None,
        gear_ratio: float = 6.33,
        kp_rotor: float = 0.0,
        kd_rotor: float = 0.03,
        control_period_s: float = 0.02,
        **kwargs: Any,
    ) -> None:
        super().__init__(hardware=True, motor_ids=motor_ids, gear_ratio=gear_ratio, **kwargs)
        self._allow_hardware = True
        self._device = device
        self._sdk_path = sdk_path
        self._kp_rotor = float(kp_rotor)
        self._kd_rotor = float(kd_rotor)
        self._period = float(control_period_s)
        self._bus: UnitreeM8010Bus | None = None
        self._io_stop = threading.Event()
        self._io_thread: threading.Thread | None = None
        self._io_error: str | None = None
        self._feedback: dict[int, MotorFeedback] = {}
        self._capture_diagnostic: dict[str, Any] | None = None

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            result = super().snapshot()
            measured_q = None
            if self._calibration is not None and all(
                mid in self._feedback for mid in self._motor_ids
            ):
                measured_q = self._calibration.rotor_to_joint(
                    [self._feedback[mid].q_rad for mid in self._motor_ids]
                )
            result.update(
                {
                    "transport": "unitree_python_sdk",
                    "backend": "unitree_go_m8010_6",
                    "hardware_io": self._bus is not None,
                    "io_thread_alive": bool(self._io_thread and self._io_thread.is_alive()),
                    "io_fault": self._io_error,
                    "capture_diagnostic": copy.deepcopy(self._capture_diagnostic),
                }
            )
            for index, motor_id in enumerate(self._motor_ids):
                feedback = self._feedback.get(motor_id)
                if feedback is None:
                    continue
                joint = result["joints"][index]
                joint.update(
                    {
                        "q_rotor": feedback.q_rad,
                        "q_joint": None if measured_q is None else measured_q[index],
                        "dq_joint": self._directions[index] * feedback.dq_rad_s / self._gear_ratio,
                        "tau": feedback.tau_nm,
                        "temperature_c": feedback.temperature_c,
                        "error": feedback.error,
                    }
                )
        return result

    def command(
        self,
        name: str,
        payload: Mapping[str, Any] | None = None,
        *,
        request_id: str,
        client_id: str,
    ) -> dict[str, Any]:
        if name == "connection.connect":
            try:
                self._open_bus()
                feedback = self._bus.brake_all() if self._bus else ()
                with self._lock:
                    self._feedback = {item.motor_id: item for item in feedback}
                response = super().command(
                    name, payload, request_id=request_id, client_id=client_id
                )
                if response["accepted"]:
                    self._start_io()
                else:
                    self._close_bus()
                return response
            except Exception as error:
                self._io_error = str(error)
                self._close_bus()
                return self._rejection(request_id, "transport_fault", str(error))
        response = super().command(name, payload, request_id=request_id, client_id=client_id)
        if response["accepted"] and name in {"stop", "estop", "fault.reset"}:
            self._best_effort_brake()
        if response["accepted"] and name == "connection.disconnect":
            self._close_bus()
        return response

    def _command_zero_capture(self, payload: dict[str, Any], client_id: str) -> dict[str, Any]:
        self._require_state(ControllerState.READ_ONLY)
        if payload.get("confirm_direction") is not True:
            raise CommandRejected(
                "confirmation_required", "zero capture requires direction confirmation"
            )
        bus = self._bus
        if bus is None:
            raise CommandRejected("not_connected", "motor bus is not connected")
        sample_count = payload.get("sample_count", 200)
        if type(sample_count) is not int or not 200 <= sample_count <= 1000:
            raise CommandRejected("invalid_payload", "sample_count must be between 200 and 1000")
        self._vector(payload.get("reference_joint_rad"), "reference_joint_rad")
        self._capture_diagnostic = None
        samples: list[tuple[MotorFeedback, ...]] = []
        expected_mode = getattr(bus, "brake_mode", 0)
        for sample_index in range(1, sample_count + 1):
            frame = bus.brake_all()
            returned_ids = tuple(item.motor_id for item in frame)
            if returned_ids != self._motor_ids:
                self._reject_capture(
                    "motor_ids",
                    sample_index,
                    f"returned motor IDs {returned_ids}, expected {self._motor_ids}",
                )
            for index, item in enumerate(frame):
                label = f"{self._joint_names[index]} motor_id={item.motor_id}"
                if not all(
                    math.isfinite(value)
                    for value in (
                        item.q_rad,
                        item.dq_rad_s,
                        item.tau_nm,
                        item.temperature_c,
                    )
                ):
                    self._reject_capture(
                        "non_finite",
                        sample_index,
                        f"{label}: non-finite feedback",
                        motor_id=item.motor_id,
                    )
                if item.error:
                    self._reject_capture(
                        "motor_error",
                        sample_index,
                        f"{label}: merror={item.error}, expected 0",
                        motor_id=item.motor_id,
                    )
                if item.mode != expected_mode:
                    self._reject_capture(
                        "mode",
                        sample_index,
                        f"{label}: mode={item.mode}, expected BRAKE={expected_mode}",
                        motor_id=item.motor_id,
                    )
                if abs(item.dq_rad_s) > 1.0:
                    joint_speed_deg_s = math.degrees(abs(item.dq_rad_s) / self._gear_ratio)
                    self._reject_capture(
                        "velocity",
                        sample_index,
                        f"{label}: |dq_rotor|={abs(item.dq_rad_s):.6f} rad/s > 1.00000 rad/s; "
                        f"joint speed={joint_speed_deg_s:.6f} deg/s "
                        f"(limit={math.degrees(0.02 / self._gear_ratio):.6f} deg/s)",
                        motor_id=item.motor_id,
                    )
                if item.temperature_c > 60:
                    self._reject_capture(
                        "temperature",
                        sample_index,
                        f"{label}: temperature={item.temperature_c:g} C > 60 C",
                        motor_id=item.motor_id,
                    )
            samples.append(frame)
        spans = [
            max(frame[index].q_rad for frame in samples)
            - min(frame[index].q_rad for frame in samples)
            for index in range(len(self._motor_ids))
        ]
        for index, span in enumerate(spans):
            if span > 0.02:
                self._reject_capture(
                    "position_span",
                    sample_count,
                    f"{self._joint_names[index]} motor_id={self._motor_ids[index]}: "
                    f"rotor position span={span:.6f} rad > 0.020000 rad "
                    f"across {sample_count} samples",
                    motor_id=self._motor_ids[index],
                )
        with self._lock:
            self._q_rotor = [
                sum(frame[index].q_rad for frame in samples) / sample_count
                for index in range(len(self._motor_ids))
            ]
            self._feedback = {item.motor_id: item for item in samples[-1]}
        super()._command_zero_capture(payload, client_id)
        assert self._candidate is not None
        data = self._candidate.to_dict()
        data.pop("calibration_id")
        data["captured_at_utc"] = datetime.now(timezone.utc).isoformat()
        data["metadata"] = {
            "backend": "unitree_python_sdk",
            "capture_client_id": client_id,
            "rotor_at_reference_rad": list(self._q_rotor),
            "rotor_span_rad": spans,
            "mapping_version": 2,
        }
        self._candidate = replace(
            self._candidate,
            calibration_id=stable_id(data, prefix="cal"),
            captured_at_utc=data["captured_at_utc"],
            metadata=data["metadata"],
        )
        return {"calibration_candidate": self._candidate.to_dict()}

    def _reject_capture(
        self,
        reason: str,
        sample_index: int,
        message: str,
        *,
        motor_id: int | None = None,
    ) -> None:
        message = f"sample {sample_index}: {message}"
        self._capture_diagnostic = {
            "reason": reason,
            "sample_index": sample_index,
            "motor_id": motor_id,
            "message": message,
        }
        raise CommandRejected("capture_unstable", message)

    def _command_zero_commit(self, payload: dict[str, Any], client_id: str) -> dict[str, Any]:
        if payload.get("confirm_direction") is not True:
            raise CommandRejected(
                "confirmation_required", "confirm the tabletop mapping before commit"
            )
        result = super()._command_zero_commit(payload, client_id)
        self._update_measured_joints()
        return result

    def _update_measured_joints(self) -> None:
        if self._calibration is not None:
            self._q = list(self._calibration.rotor_to_joint(self._q_rotor))
            self._dq = [
                direction * self._feedback[mid].dq_rad_s / self._gear_ratio
                for direction, mid in zip(self._directions, self._motor_ids, strict=True)
            ]

    def _open_bus(self) -> None:
        if self._bus is None:
            config = UnitreeBusConfig(
                device=self._device,
                motor_ids=tuple(self._motor_ids),
                gear_ratio=self._gear_ratio,
                sdk_path=self._sdk_path,
            )
            self._bus = UnitreeM8010Bus(config)

    def _start_io(self) -> None:
        if self._io_thread and self._io_thread.is_alive():
            return
        self._io_stop.clear()
        self._io_thread = threading.Thread(target=self._io_loop, name="qarm-m8010", daemon=True)
        self._io_thread.start()

    def _io_loop(self) -> None:
        while not self._io_stop.is_set():
            started = time.monotonic()
            try:
                self._io_cycle()
            except Exception as error:
                self._io_error = str(error)
                self._enter_fault(f"motor I/O failure: {error}")
                self._best_effort_brake()
                return
            self._io_stop.wait(max(0.0, self._period - (time.monotonic() - started)))

    def _io_cycle(self) -> None:
        # Capture/commit and feedback conversion must use one calibration epoch.
        with self._lock:
            self._io_cycle_locked()

    def _io_cycle_locked(self) -> None:
        bus = self._bus
        if bus is None:
            return
        with self._lock:
            state = self._state
            calibration = self._calibration
            q, dq = tuple(self._q), tuple(self._dq)
            directions = tuple(self._directions)
            rotor_targets = None if calibration is None else calibration.joint_to_rotor(q)
        active = (
            state
            in {ControllerState.READY, ControllerState.GRAVITY_HOLD, ControllerState.EXECUTING}
            and calibration is not None
        )
        feedback = []
        for index, motor_id in enumerate(self._motor_ids):
            if active:
                command = MotorCommand(
                    motor_id=motor_id,
                    q_rad=rotor_targets[index],
                    dq_rad_s=directions[index] * self._gear_ratio * dq[index],
                    kp=self._kp_rotor,
                    kd=self._kd_rotor,
                    mode="FOC",
                )
            else:
                command = MotorCommand(motor_id=motor_id, mode="BRAKE")
            feedback.append(bus.exchange(command))
        with self._lock:
            self._feedback = {item.motor_id: item for item in feedback}
            for index, item in enumerate(feedback):
                self._q_rotor[index] = item.q_rad
            self._update_measured_joints()
            self._sequence += 1

    def _best_effort_brake(self) -> None:
        if self._bus is None:
            return
        with suppress(Exception):
            self._bus.brake_all()

    def _close_bus(self) -> None:
        self._io_stop.set()
        if self._io_thread and self._io_thread is not threading.current_thread():
            self._io_thread.join(timeout=2.0)
        self._io_thread = None
        bus, self._bus = self._bus, None
        if bus is not None:
            bus.close()

    def close(self) -> None:
        self._close_bus()
