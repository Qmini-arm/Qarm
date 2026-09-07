"""A deterministic, in-memory control backend for Viser and protocol tests.

``FakeArmBackend`` models controller ownership and safety decisions, but never
opens a serial device or creates a motor SDK command.  It is deliberately a
high-level backend: clients submit named actions and planned trajectories; the
backend advances the simulated arm from its own clock.
"""

from __future__ import annotations

import bisect
import copy
import json
import math
import threading
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import replace
from typing import Any

from .schemas import (
    ArmSnapshot,
    CalibrationManifest,
    CalibrationState,
    ControllerState,
    JointSnapshot,
    PlanRecord,
    stable_id,
)

SCHEMA_VERSION = 1


class CommandRejected(ValueError):
    """Expected rejection of a command at the control boundary."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


CollisionValidator = Callable[[tuple[tuple[float, ...], ...]], bool]


def _default_joint_names(dof: int) -> tuple[str, ...]:
    return tuple(f"joint_{index}" for index in range(1, dof + 1))


def _default_collision_validator(
    positions: tuple[tuple[float, ...], ...],
) -> bool:
    """Fail closed until the caller supplies the active model's collision check."""
    del positions
    raise CommandRejected("collision_checker_required", "backend collision validation is not configured")


class FakeArmBackend:
    """Testable control-domain implementation with no hardware side effects.

    Parameters ``within_limits`` and ``collision_validator`` are supplied by
    the model/planning layer.  Without an injected collision check, plan
    validation fails closed.  Gravity mode is a state-machine simulation only:
    it holds the pose and reports zero simulated effort, without physical dynamics.
    """

    _LEASE_COMMANDS = frozenset(
        {
            "zero.capture",
            "zero.commit",
            "control.enable",
            "gravity.set",
            "plan.validate",
            "plan.execute",
        }
    )
    _COMMANDS = frozenset(
        {
            "connection.connect",
            "connection.disconnect",
            "lease.acquire",
            "lease.heartbeat",
            "lease.release",
            "zero.capture",
            "zero.commit",
            "control.enable",
            "gravity.set",
            "plan.validate",
            "plan.execute",
            "stop",
            "estop",
            "fault.reset",
        }
    )

    def __init__(
        self,
        *,
        model_hash: str,
        joint_names: Sequence[str] | None = None,
        motor_ids: Sequence[int] | None = None,
        joint_limits_rad: Sequence[Sequence[float]] | None = None,
        board_boot_id: str = "fake-board-boot-1",
        collision_validator: CollisionValidator | Any | None = None,
        within_limits: Callable[[Sequence[float]], bool] | None = None,
        hardware: bool = False,
        lease_timeout_s: float = 5.0,
        start_tolerance_rad: float = 0.02,
        velocity_limits_rad_s: Sequence[float] | None = None,
        gear_ratio: float = 6.33,
        clock_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        if not isinstance(model_hash, str) or not model_hash.strip():
            raise ValueError("model_hash must be a non-empty string")
        names = tuple(joint_names or _default_joint_names(4))
        if not names or len(set(names)) != len(names) or any(not str(name).strip() for name in names):
            raise ValueError("joint_names must be non-empty and unique")
        ids = tuple(range(len(names))) if motor_ids is None else tuple(int(item) for item in motor_ids)
        if len(ids) != len(names) or len(set(ids)) != len(ids) or any(item < 0 or item > 14 for item in ids):
            raise ValueError("motor_ids must be unique and in [0, 14]")
        limits = (
            tuple((-math.pi, math.pi) for _ in names)
            if joint_limits_rad is None
            else tuple((float(pair[0]), float(pair[1])) for pair in joint_limits_rad)
        )
        if len(limits) != len(names) or any(
            not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper
            for lower, upper in limits
        ):
            raise ValueError("joint_limits_rad must define finite lower/upper bounds per joint")
        if not math.isfinite(lease_timeout_s) or lease_timeout_s <= 0:
            raise ValueError("lease_timeout_s must be positive and finite")
        if not math.isfinite(start_tolerance_rad) or start_tolerance_rad < 0:
            raise ValueError("start_tolerance_rad must be non-negative and finite")

        self._lock = threading.RLock()
        self._model_hash = model_hash.strip()
        self._joint_names = names
        self._motor_ids = ids
        self._limits = limits
        self._velocity_limits = tuple(
            float(item) for item in (
                velocity_limits_rad_s if velocity_limits_rad_s is not None else [0.5] * len(names)
            )
        )
        if len(self._velocity_limits) != len(names) or any(
            not math.isfinite(item) or item <= 0 for item in self._velocity_limits
        ):
            raise ValueError("velocity_limits_rad_s must contain a positive finite limit per joint")
        self._board_boot_id = str(board_boot_id).strip()
        if not self._board_boot_id:
            raise ValueError("board_boot_id must be a non-empty string")
        self._hardware = bool(hardware)
        if not math.isfinite(gear_ratio) or gear_ratio <= 0:
            raise ValueError("gear_ratio must be positive and finite")
        self._gear_ratio = float(gear_ratio)
        self._allow_hardware = False
        self._lease_timeout_s = float(lease_timeout_s)
        self._start_tolerance_rad = float(start_tolerance_rad)
        self._clock_ns = clock_ns
        self._epoch_ns = clock_ns()
        self._within_limits = within_limits or self._limits_allow
        self._collision_validator = self._resolve_collision_validator(collision_validator)
        self._state = ControllerState.DISCONNECTED
        self._sequence = 0
        self._sim_time_s = 0.0
        self._q = [0.0] * len(names)
        self._dq = [0.0] * len(names)
        self._q_rotor = [0.0] * len(names)
        self._directions = [1] * len(names)
        self._zero_offsets = [0.0] * len(names)
        self._calibration: CalibrationManifest | None = None
        self._candidate: CalibrationManifest | None = None
        self._plans: dict[str, PlanRecord] = {}
        self._active_plan: PlanRecord | None = None
        self._execution_elapsed_s = 0.0
        self._gravity_scale = 0.0
        self._fault: str | None = None
        self._lease_client_id: str | None = None
        self._lease_until_s = 0.0
        self._responses: dict[str, dict[str, Any]] = {}
        self._requests: dict[str, str] = {}

    def snapshot(self) -> dict[str, Any]:
        """Return the current snapshot.  This method has no external I/O."""
        with self._lock:
            self._expire_lease()
            snapshot = self._arm_snapshot().to_dict()
            snapshot.update(
                {
                    "schema_version": SCHEMA_VERSION,
                    "transport": "hardware" if self._hardware else "fake",
                    "backend": "kinematic_simulation",
                    "hardware_io": False,
                    "calibration_state": (
                        CalibrationState.VALID.value
                        if self._calibration is not None
                        else CalibrationState.CAPTURED.value
                        if self._candidate is not None
                        else CalibrationState.UNCALIBRATED.value
                    ),
                    "calibration_candidate_id": (
                        None if self._candidate is None else self._candidate.calibration_id
                    ),
                    "calibration_candidate": (
                        None if self._candidate is None else self._candidate.to_dict()
                    ),
                    "calibration": (
                        None if self._calibration is None else self._calibration.to_dict()
                    ),
                    "lease_expires_monotonic_ns": self._lease_expiry_ns(),
                }
            )
            if snapshot["lease"] is not None:
                snapshot["lease"]["expires_monotonic_ns"] = self._lease_expiry_ns()
            return snapshot

    def command(
        self,
        name: str,
        payload: Mapping[str, Any] | None = None,
        *,
        request_id: str,
        client_id: str,
    ) -> dict[str, Any]:
        """Apply one high-level command and return a cached, serializable result.

        A ``request_id`` is globally idempotent.  A replay returns the original
        response even after the backend state has moved on, which prevents a
        retry from executing a plan or committing calibration twice.
        """
        with self._lock:
            if not isinstance(request_id, str) or not request_id.strip():
                return self._rejection("", "invalid_request", "request_id must be a non-empty string")
            try:
                fingerprint = json.dumps(
                    {"name": name, "payload": payload or {}, "client_id": client_id},
                    sort_keys=True, allow_nan=False,
                )
            except (TypeError, ValueError):
                return self._rejection(request_id, "invalid_payload", "payload must be finite JSON data")
            if request_id in self._responses:
                if self._requests[request_id] != fingerprint:
                    return self._rejection(request_id, "request_id_conflict", "request_id was already used for a different command or client")
                return self._copy_response(self._responses[request_id])
            try:
                if not isinstance(name, str) or name not in self._COMMANDS:
                    raise CommandRejected("unknown_command", f"unsupported command: {name!r}")
                if not isinstance(client_id, str) or not client_id.strip():
                    raise CommandRejected("invalid_client", "client_id must be a non-empty string")
                if payload is None:
                    payload = {}
                if not isinstance(payload, Mapping):
                    raise CommandRejected("invalid_payload", "payload must be an object")
                self._expire_lease()
                if self._hardware and not self._allow_hardware:
                    raise CommandRejected(
                        "hardware_adapter_unavailable",
                        "hardware control requires HardwareArmBackend",
                    )
                if name in self._LEASE_COMMANDS:
                    self._require_lease(client_id)
                handler = getattr(self, "_command_" + name.replace(".", "_"))
                details = handler(dict(payload), client_id)
                self._sequence += 1
                response = self._response(request_id, accepted=True, details=details)
            except CommandRejected as exc:
                response = self._rejection(request_id, exc.code, str(exc))
            except (TypeError, ValueError) as exc:
                response = self._rejection(request_id, "invalid_payload", str(exc))
            self._responses[request_id] = response
            self._requests[request_id] = fingerprint
            return self._copy_response(response)

    def advance(self, dt: float) -> dict[str, Any]:
        """Advance fake time and interpolate every sample of an executing plan."""
        with self._lock:
            value = float(dt)
            if not math.isfinite(value) or value < 0:
                raise ValueError("dt must be finite and non-negative")
            allowed_dt = value
            if self._lease_client_id is not None:
                allowed_dt = min(value, max(0.0, self._lease_until_s - self._sim_time_s))
            if self._state is ControllerState.EXECUTING and self._active_plan is not None:
                self._execution_elapsed_s += allowed_dt
                self._apply_plan_at(self._active_plan, self._execution_elapsed_s)
                self._sequence += 1
                if self._execution_elapsed_s >= self._active_plan.times_s[-1]:
                    self._active_plan = None
                    self._execution_elapsed_s = 0.0
                    self._dq = [0.0] * len(self._q)
                    self._state = ControllerState.READY
            self._sim_time_s += value
            self._expire_lease()
            return self.snapshot()

    def inject_fault(self, reason: str) -> None:
        """Testing hook for an asynchronous controller fault; never hardware I/O."""
        with self._lock:
            message = str(reason).strip()
            if not message:
                raise ValueError("fault reason must not be empty")
            self._enter_fault(message)

    def _command_connection_connect(self, payload: dict[str, Any], client_id: str) -> dict[str, Any]:
        del payload, client_id
        if self._state is not ControllerState.DISCONNECTED:
            raise CommandRejected("invalid_state", "connection is already established")
        self._state = ControllerState.READ_ONLY
        return {"connected": True}

    def _command_connection_disconnect(self, payload: dict[str, Any], client_id: str) -> dict[str, Any]:
        del payload, client_id
        self._require_connected()
        if self._state in (
            ControllerState.EXECUTING, ControllerState.GRAVITY_HOLD,
            ControllerState.ESTOP, ControllerState.FAULT,
        ):
            raise CommandRejected("invalid_state", "stop or explicitly reset the latched fault before disconnecting")
        self._lease_client_id = None
        self._lease_until_s = 0.0
        self._state = ControllerState.DISCONNECTED
        return {"connected": False}

    def _command_lease_acquire(self, payload: dict[str, Any], client_id: str) -> dict[str, Any]:
        self._require_connected()
        if self._state in (ControllerState.FAULT, ControllerState.ESTOP):
            raise CommandRejected("invalid_state", "cannot acquire a lease while faulted or stopped")
        requested = payload.get("timeout_s", self._lease_timeout_s)
        try:
            timeout_s = float(requested)
        except (TypeError, ValueError) as exc:
            raise CommandRejected("invalid_payload", "timeout_s must be numeric") from exc
        if not math.isfinite(timeout_s) or timeout_s <= 0 or timeout_s > 30:
            raise CommandRejected("invalid_payload", "timeout_s must be in (0, 30]")
        if self._lease_client_id not in (None, client_id):
            raise CommandRejected("lease_held", "another client owns the control lease")
        self._lease_client_id = client_id
        self._lease_until_s = self._sim_time_s + timeout_s
        return {"lease_client_id": client_id, "timeout_s": timeout_s}

    def _command_lease_heartbeat(self, payload: dict[str, Any], client_id: str) -> dict[str, Any]:
        del payload
        self._require_lease(client_id)
        self._lease_until_s = self._sim_time_s + self._lease_timeout_s
        return {"lease_client_id": client_id, "timeout_s": self._lease_timeout_s}

    def _command_lease_release(self, payload: dict[str, Any], client_id: str) -> dict[str, Any]:
        del payload
        self._require_lease(client_id)
        if self._state in (ControllerState.EXECUTING, ControllerState.GRAVITY_HOLD):
            raise CommandRejected("invalid_state", "stop before releasing the active control lease")
        self._lease_client_id = None
        self._lease_until_s = 0.0
        return {"released": True}

    def _command_zero_capture(self, payload: dict[str, Any], client_id: str) -> dict[str, Any]:
        self._require_state(ControllerState.READ_ONLY)
        reference = self._vector(payload.get("reference_joint_rad", self._q), "reference_joint_rad")
        directions_raw = payload.get("directions", self._directions)
        try:
            directions = tuple(int(item) for item in directions_raw)
        except (TypeError, ValueError) as exc:
            raise CommandRejected("invalid_payload", "directions must contain integers") from exc
        if len(directions) != len(self._q) or any(item not in (-1, 1) for item in directions):
            raise CommandRejected("invalid_payload", "directions must contain +1 or -1 per joint")
        sample_count = payload.get("sample_count", 200)
        if type(sample_count) is not int or sample_count < 200:
            raise CommandRejected("invalid_payload", "sample_count must be an integer of at least 200")
        # Capture represents a manually supported fixture pose.  No motor command is issued.
        self._q = list(reference)
        self._calibration = None
        self._plans.clear()
        self._dq = [0.0] * len(self._q)
        self._directions = list(directions)
        self._zero_offsets = [
            rotor - direction * self._gear_ratio * joint
            for rotor, direction, joint in zip(self._q_rotor, directions, reference)
        ]
        candidate_data = {
            "model_hash": self._model_hash,
            "board_boot_id": self._board_boot_id,
            "joint_names": list(self._joint_names),
            "motor_ids": list(self._motor_ids),
            "directions": list(directions),
            "zero_offsets_rad": list(self._zero_offsets),
            "reference_joint_rad": list(reference),
            "gear_ratio": self._gear_ratio,
            "sample_count": sample_count,
            "captured_at_utc": "1970-01-01T00:00:00+00:00",
        }
        candidate_id = stable_id(candidate_data, prefix="cal")
        self._candidate = CalibrationManifest(
            calibration_id=candidate_id,
            state=CalibrationState.CAPTURED,
            metadata={"backend": "fake", "capture_client_id": client_id},
            **candidate_data,
        )
        self._state = ControllerState.ZERO_CAPTURE
        return {"calibration_candidate": self._candidate.to_dict()}

    def _command_zero_commit(self, payload: dict[str, Any], client_id: str) -> dict[str, Any]:
        del client_id
        self._require_state(ControllerState.ZERO_CAPTURE)
        if self._candidate is None:
            raise CommandRejected("no_candidate", "no zero capture is available to commit")
        candidate_id = payload.get("calibration_id")
        if candidate_id != self._candidate.calibration_id:
            raise CommandRejected("calibration_mismatch", "calibration_id does not match the captured candidate")
        self._calibration = replace(self._candidate, state=CalibrationState.VALID)
        self._candidate = None
        self._plans.clear()
        self._state = ControllerState.CALIBRATION_VALID
        return {"calibration": self._calibration.to_dict()}

    def _command_control_enable(self, payload: dict[str, Any], client_id: str) -> dict[str, Any]:
        del client_id
        enabled = payload.get("enabled", True)
        if type(enabled) is not bool:
            raise CommandRejected("invalid_payload", "enabled must be boolean")
        if not enabled:
            if self._state not in (
                ControllerState.CALIBRATION_VALID,
                ControllerState.READY,
                ControllerState.GRAVITY_HOLD,
            ):
                raise CommandRejected("invalid_state", "control cannot be disabled in the current state")
            self._active_plan = None
            self._gravity_scale = 0.0
            self._state = ControllerState.READ_ONLY
            return {"enabled": False}
        self._require_calibration()
        if self._state not in (ControllerState.CALIBRATION_VALID, ControllerState.READ_ONLY):
            raise CommandRejected("invalid_state", "control can only be enabled from calibrated read-only state")
        if not self._within_limits(self._q):
            raise CommandRejected("joint_limits", "current pose is outside active soft limits")
        self._state = ControllerState.READY
        return {"enabled": True}

    def _command_gravity_set(self, payload: dict[str, Any], client_id: str) -> dict[str, Any]:
        del client_id
        enabled = payload.get("enabled", True)
        if type(enabled) is not bool:
            raise CommandRejected("invalid_payload", "enabled must be boolean")
        if enabled:
            self._require_calibration()
            self._require_state(ControllerState.READY)
            scale = payload.get("scale", 1.0)
            try:
                scale = float(scale)
            except (TypeError, ValueError) as exc:
                raise CommandRejected("invalid_payload", "scale must be numeric") from exc
            if not math.isfinite(scale) or not 0.0 <= scale <= 1.0:
                raise CommandRejected("invalid_payload", "scale must be within [0, 1]")
            self._gravity_scale = scale
            self._state = ControllerState.GRAVITY_HOLD
        else:
            self._require_state(ControllerState.GRAVITY_HOLD)
            self._gravity_scale = 0.0
            self._state = ControllerState.READY
        return {"enabled": enabled, "scale": self._gravity_scale}

    def _command_plan_validate(self, payload: dict[str, Any], client_id: str) -> dict[str, Any]:
        del client_id
        self._require_state(ControllerState.READY)
        calibration = self._require_calibration()
        plan = self._plan_from_payload(payload)
        if plan.model_hash != self._model_hash or plan.calibration_id != calibration.calibration_id:
            raise CommandRejected("identity_mismatch", "plan model_hash or calibration_id is not active")
        if plan.times_s[0] != 0.0:
            raise CommandRejected("invalid_plan", "plan must begin at time 0")
        if not self._start_matches(plan.positions_rad[0]):
            raise CommandRejected("start_mismatch", "plan start does not match current joint position")
        if any(not self._within_limits(row) for row in plan.positions_rad):
            raise CommandRejected("joint_limits", "plan contains a position outside active soft limits")
        self._validate_plan_velocity(plan)
        if not self._path_is_collision_free(plan.positions_rad):
            raise CommandRejected("collision", "backend collision validation rejected the plan")
        approved = replace(plan, collision_checked=True, validated=True)
        self._plans[approved.plan_id] = approved
        return {"plan": approved.to_dict()}

    def _command_plan_execute(self, payload: dict[str, Any], client_id: str) -> dict[str, Any]:
        del client_id
        self._require_state(ControllerState.READY)
        calibration = self._require_calibration()
        plan_id = payload.get("plan_id")
        if not isinstance(plan_id, str) or not plan_id:
            raise CommandRejected("invalid_payload", "plan_id must be a non-empty string")
        plan = self._plans.get(plan_id)
        if plan is None:
            raise CommandRejected("unknown_plan", "plan_id has not been validated by this backend")
        try:
            plan.validate_for(model_hash=self._model_hash, calibration_id=calibration.calibration_id)
        except ValueError as exc:
            raise CommandRejected("identity_mismatch", str(exc)) from exc
        if not self._start_matches(plan.positions_rad[0]):
            raise CommandRejected("start_mismatch", "plan start no longer matches current joint position")
        if payload.get("model_hash", self._model_hash) != self._model_hash or payload.get(
            "calibration_id", calibration.calibration_id
        ) != calibration.calibration_id:
            raise CommandRejected("identity_mismatch", "execution identity is not active")
        if not self._path_is_collision_free(plan.positions_rad):
            raise CommandRejected("collision", "backend collision revalidation rejected the plan")
        self._active_plan = plan
        self._execution_elapsed_s = 0.0
        self._state = ControllerState.EXECUTING
        self._apply_plan_at(plan, 0.0)
        return {"plan_id": plan.plan_id, "duration_s": plan.times_s[-1]}

    def _command_stop(self, payload: dict[str, Any], client_id: str) -> dict[str, Any]:
        del payload, client_id
        self._require_connected()
        if self._state in (ControllerState.DISCONNECTED, ControllerState.FAULT, ControllerState.ESTOP):
            raise CommandRejected("invalid_state", "stop is unavailable in the current state")
        self._active_plan = None
        self._execution_elapsed_s = 0.0
        self._gravity_scale = 0.0
        self._dq = [0.0] * len(self._dq)
        self._candidate = None
        self._state = (
            ControllerState.READY
            if self._state in (ControllerState.READY, ControllerState.EXECUTING, ControllerState.GRAVITY_HOLD)
            else ControllerState.CALIBRATION_VALID
            if self._calibration is not None
            else ControllerState.READ_ONLY
        )
        return {"stopped": True}

    def _command_estop(self, payload: dict[str, Any], client_id: str) -> dict[str, Any]:
        del payload, client_id
        self._require_connected()
        self._active_plan = None
        self._execution_elapsed_s = 0.0
        self._gravity_scale = 0.0
        self._dq = [0.0] * len(self._dq)
        self._lease_client_id = None
        self._lease_until_s = 0.0
        self._state = ControllerState.ESTOP
        return {"estop": True}

    def _command_fault_reset(self, payload: dict[str, Any], client_id: str) -> dict[str, Any]:
        del payload, client_id
        self._require_connected()
        if self._state not in (ControllerState.FAULT, ControllerState.ESTOP):
            raise CommandRejected("invalid_state", "fault.reset requires a fault or estop state")
        self._fault = None
        self._active_plan = None
        self._gravity_scale = 0.0
        self._state = ControllerState.READ_ONLY
        self._lease_client_id = None
        self._lease_until_s = 0.0
        return {"reset": True}

    def _plan_from_payload(self, payload: Mapping[str, Any]) -> PlanRecord:
        expected = {"times_s", "positions_rad", "velocities_rad_s", "model_hash", "calibration_id"}
        missing = expected - set(payload)
        if missing:
            raise CommandRejected("invalid_payload", "plan payload is missing " + ", ".join(sorted(missing)))
        raw = {key: payload[key] for key in expected}
        plan_id = stable_id(raw, prefix="plan")
        try:
            plan = PlanRecord(plan_id=plan_id, **raw)
        except (TypeError, ValueError) as exc:
            raise CommandRejected("invalid_plan", str(exc)) from exc
        if len(plan.positions_rad[0]) != len(self._joint_names):
            raise CommandRejected("invalid_plan", "plan degree of freedom does not match active model")
        return plan

    def _path_is_collision_free(self, positions: tuple[tuple[float, ...], ...]) -> bool:
        try:
            return bool(self._collision_validator(positions))
        except CommandRejected:
            raise
        except (TypeError, ValueError) as exc:
            raise CommandRejected("collision_validation_error", str(exc)) from exc

    def _resolve_collision_validator(self, validator: CollisionValidator | Any | None) -> CollisionValidator:
        if validator is None:
            return _default_collision_validator
        if callable(validator):
            return validator
        if hasattr(validator, "path_is_free"):
            return lambda positions: bool(
                validator.is_free(positions[0]) and validator.path_is_free(positions)
            )
        raise ValueError("collision_validator must be callable or expose path_is_free")

    def _validate_plan_velocity(self, plan: PlanRecord) -> None:
        for velocity in plan.velocities_rad_s:
            if any(abs(value) > limit + 1e-9 for value, limit in zip(velocity, self._velocity_limits)):
                raise CommandRejected("velocity_limit", "plan velocity exceeds an active joint limit")
        for index in range(1, len(plan.times_s)):
            duration = plan.times_s[index] - plan.times_s[index - 1]
            if any(
                abs(end - start) / duration > limit + 1e-9
                for start, end, limit in zip(
                    plan.positions_rad[index - 1], plan.positions_rad[index], self._velocity_limits
                )
            ):
                raise CommandRejected("velocity_limit", "plan segment exceeds an active joint speed limit")

    def _apply_plan_at(self, plan: PlanRecord, elapsed_s: float) -> None:
        times = plan.times_s
        if elapsed_s >= times[-1]:
            self._set_joint_state(plan.positions_rad[-1], plan.velocities_rad_s[-1])
            return
        upper = bisect.bisect_right(times, elapsed_s)
        if upper == 0:
            self._set_joint_state(plan.positions_rad[0], plan.velocities_rad_s[0])
            return
        lower = upper - 1
        t0, t1 = times[lower], times[upper]
        fraction = (elapsed_s - t0) / (t1 - t0)
        position = tuple(
            start + fraction * (end - start)
            for start, end in zip(plan.positions_rad[lower], plan.positions_rad[upper])
        )
        # The fake follows piecewise-linear positions. Report their actual slope;
        # supplied trajectory velocity samples are independently safety-checked.
        velocity = tuple(
            (end - start) / (t1 - t0)
            for start, end in zip(plan.positions_rad[lower], plan.positions_rad[upper])
        )
        self._set_joint_state(position, velocity)

    def _set_joint_state(self, position: Sequence[float], velocity: Sequence[float]) -> None:
        self._q = list(position)
        self._dq = list(velocity)
        self._q_rotor = [
            zero + direction * self._gear_ratio * joint
            for zero, direction, joint in zip(self._zero_offsets, self._directions, self._q)
        ]

    def _arm_snapshot(self) -> ArmSnapshot:
        calibrated = self._calibration is not None
        joints = tuple(
            JointSnapshot(
                name=name,
                motor_id=motor_id,
                q_rotor_rad=self._q_rotor[index],
                q_joint_rad=self._q[index] if calibrated else None,
                dq_joint_rad_s=self._dq[index],
                tau_nm=0.0,
                temperature_c=30.0,
            )
            for index, (name, motor_id) in enumerate(zip(self._joint_names, self._motor_ids))
        )
        return ArmSnapshot(
            sequence=self._sequence,
            monotonic_ns=self._epoch_ns + int(self._sim_time_s * 1_000_000_000),
            state=self._state,
            model_hash=self._model_hash,
            calibration_id=None if self._calibration is None else self._calibration.calibration_id,
            board_boot_id=self._board_boot_id,
            joints=joints,
            gravity_scale=self._gravity_scale,
            active_plan_id=None if self._active_plan is None else self._active_plan.plan_id,
            fault=self._fault,
            lease_client_id=self._lease_client_id,
        )

    def _response(self, request_id: str, *, accepted: bool, details: Mapping[str, Any]) -> dict[str, Any]:
        return {
            "schema_version": SCHEMA_VERSION,
            "request_id": request_id,
            "accepted": accepted,
            "error": None,
            "result": dict(details),
            "snapshot": self.snapshot(),
        }

    def _rejection(self, request_id: str, code: str, message: str) -> dict[str, Any]:
        return {
            "schema_version": SCHEMA_VERSION,
            "request_id": request_id,
            "accepted": False,
            "error": {"code": code, "message": message},
            "result": None,
            "snapshot": self.snapshot(),
        }

    @staticmethod
    def _copy_response(response: Mapping[str, Any]) -> dict[str, Any]:
        # JSON round trips are avoided, but nested mutable data cannot escape the cache.
        return copy.deepcopy(dict(response))

    def _require_connected(self) -> None:
        if self._state is ControllerState.DISCONNECTED:
            raise CommandRejected("not_connected", "connection.connect is required first")

    def _require_state(self, *states: ControllerState) -> None:
        self._require_connected()
        if self._state not in states:
            required = ", ".join(state.value for state in states)
            raise CommandRejected("invalid_state", f"command requires state: {required}")

    def _require_lease(self, client_id: str) -> None:
        self._require_connected()
        if self._lease_client_id != client_id or self._lease_until_s <= self._sim_time_s:
            raise CommandRejected("lease_required", "an active control lease is required")

    def _require_calibration(self) -> CalibrationManifest:
        if self._calibration is None:
            raise CommandRejected("calibration_required", "a valid calibration is required")
        try:
            self._calibration.validate_for(
                model_hash=self._model_hash,
                board_boot_id=self._board_boot_id,
                joint_names=self._joint_names,
                motor_ids=self._motor_ids,
            )
        except ValueError as exc:
            raise CommandRejected("calibration_invalid", str(exc)) from exc
        return self._calibration

    def _start_matches(self, start: Sequence[float]) -> bool:
        return all(
            abs(float(value) - current) <= self._start_tolerance_rad
            for value, current in zip(start, self._q)
        )

    def _vector(self, value: Any, label: str) -> tuple[float, ...]:
        try:
            vector = tuple(float(item) for item in value)
        except (TypeError, ValueError) as exc:
            raise CommandRejected("invalid_payload", f"{label} must be a numeric vector") from exc
        if len(vector) != len(self._q) or not all(math.isfinite(item) for item in vector):
            raise CommandRejected("invalid_payload", f"{label} must contain finite values per joint")
        return vector

    def _limits_allow(self, values: Sequence[float]) -> bool:
        return len(values) == len(self._limits) and all(
            lower <= float(value) <= upper
            for value, (lower, upper) in zip(values, self._limits)
        )

    def _expire_lease(self) -> None:
        if self._lease_client_id is not None and self._lease_until_s <= self._sim_time_s:
            self._lease_client_id = None
            self._lease_until_s = 0.0
            if self._state in (ControllerState.EXECUTING, ControllerState.GRAVITY_HOLD):
                self._active_plan = None
                self._execution_elapsed_s = 0.0
                self._gravity_scale = 0.0
                self._dq = [0.0] * len(self._dq)
                self._state = ControllerState.FAULT
                self._fault = "control lease expired during active control"
                self._sequence += 1

    def _lease_expiry_ns(self) -> int | None:
        if self._lease_client_id is None:
            return None
        return self._epoch_ns + int(self._lease_until_s * 1_000_000_000)

    def _enter_fault(self, reason: str) -> None:
        self._fault = reason
        self._active_plan = None
        self._execution_elapsed_s = 0.0
        self._gravity_scale = 0.0
        self._dq = [0.0] * len(self._dq)
        self._state = ControllerState.FAULT
        self._sequence += 1
