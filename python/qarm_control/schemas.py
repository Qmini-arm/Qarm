"""Small, dependency-free records shared by the control front ends.

The records in this module deliberately contain only control-domain data.  In
particular, they do not contain SDK objects or file handles, which keeps them
safe to pass over the future socket protocol and straightforward to use in
simulation tests.
"""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Any


def _finite_vector(
    value: Sequence[float], *, name: str, size: int | None = None
) -> tuple[float, ...]:
    values = tuple(float(item) for item in value)
    if size is not None and len(values) != size:
        raise ValueError(f"{name} must contain {size} values")
    if not all(math.isfinite(item) for item in values):
        raise ValueError(f"{name} must contain finite values")
    return values


def _required_text(value: str, *, name: str) -> str:
    text = str(value).strip()
    if not text:
        raise ValueError(f"{name} must not be empty")
    return text


class ControllerState(str, Enum):
    DISCONNECTED = "disconnected"
    READ_ONLY = "read_only"
    ZERO_CAPTURE = "zero_capture"
    CALIBRATION_VALID = "calibration_valid"
    READY = "ready"
    GRAVITY_HOLD = "gravity_hold"
    EXECUTING = "executing"
    FAULT = "fault"
    ESTOP = "estop"


class CalibrationState(str, Enum):
    UNCALIBRATED = "uncalibrated"
    CAPTURED = "captured"
    VALID = "valid"


@dataclass(frozen=True, slots=True)
class CalibrationManifest:
    """Mapping of measured rotor radians to URDF joint radians.

    ``zero_offsets_rad`` are rotor readings at *joint mathematical zero*, not
    the readings measured at the tabletop reference. The latter are recovered
    by adding ``direction * gear_ratio * reference_joint_rad``.
    """

    calibration_id: str
    model_hash: str
    board_boot_id: str
    joint_names: tuple[str, ...]
    motor_ids: tuple[int, ...]
    directions: tuple[int, ...]
    zero_offsets_rad: tuple[float, ...]
    reference_joint_rad: tuple[float, ...]
    sample_count: int
    captured_at_utc: str
    state: CalibrationState = CalibrationState.VALID
    metadata: Mapping[str, Any] = field(default_factory=dict)
    gear_ratio: float = 6.33

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "calibration_id", _required_text(self.calibration_id, name="calibration_id")
        )
        object.__setattr__(self, "model_hash", _required_text(self.model_hash, name="model_hash"))
        object.__setattr__(
            self, "board_boot_id", _required_text(self.board_boot_id, name="board_boot_id")
        )
        names = tuple(_required_text(name, name="joint_name") for name in self.joint_names)
        ids = tuple(int(item) for item in self.motor_ids)
        directions = tuple(int(item) for item in self.directions)
        if not names or len(set(names)) != len(names):
            raise ValueError("joint_names must be non-empty and unique")
        if (
            len(ids) != len(names)
            or len(set(ids)) != len(ids)
            or any(item < 0 or item > 14 for item in ids)
        ):
            raise ValueError("motor_ids must contain unique IDs in [0, 14]")
        if len(directions) != len(names) or any(item not in (-1, 1) for item in directions):
            raise ValueError("directions must contain one value of +1 or -1 per joint")
        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "motor_ids", ids)
        object.__setattr__(self, "directions", directions)
        object.__setattr__(
            self,
            "zero_offsets_rad",
            _finite_vector(self.zero_offsets_rad, name="zero_offsets_rad", size=len(names)),
        )
        object.__setattr__(
            self,
            "reference_joint_rad",
            _finite_vector(self.reference_joint_rad, name="reference_joint_rad", size=len(names)),
        )
        if type(self.sample_count) is not int or self.sample_count < 20:
            raise ValueError("sample_count must be at least 20")
        object.__setattr__(
            self, "captured_at_utc", _required_text(self.captured_at_utc, name="captured_at_utc")
        )
        try:
            datetime.fromisoformat(self.captured_at_utc.replace("Z", "+00:00"))
        except ValueError as exc:
            raise ValueError("captured_at_utc must be an ISO-8601 timestamp") from exc
        if not isinstance(self.state, CalibrationState):
            object.__setattr__(self, "state", CalibrationState(self.state))
        object.__setattr__(self, "metadata", dict(self.metadata))
        if not math.isfinite(self.gear_ratio) or self.gear_ratio <= 0:
            raise ValueError("gear_ratio must be positive and finite")

    @property
    def rotor_at_reference_rad(self) -> tuple[float, ...]:
        return tuple(
            zero + direction * self.gear_ratio * reference
            for zero, direction, reference in zip(
                self.zero_offsets_rad, self.directions, self.reference_joint_rad, strict=True
            )
        )

    def rotor_to_joint(self, rotor_rad: Sequence[float]) -> tuple[float, ...]:
        rotor = _finite_vector(rotor_rad, name="rotor_rad", size=len(self.joint_names))
        return tuple(
            reference + direction * (value - measured) / self.gear_ratio
            for reference, direction, value, measured in zip(
                self.reference_joint_rad,
                self.directions,
                rotor,
                self.rotor_at_reference_rad,
                strict=True,
            )
        )

    def joint_to_rotor(self, joint_rad: Sequence[float]) -> tuple[float, ...]:
        joint = _finite_vector(joint_rad, name="joint_rad", size=len(self.joint_names))
        return tuple(
            measured + direction * self.gear_ratio * (value - reference)
            for measured, direction, value, reference in zip(
                self.rotor_at_reference_rad,
                self.directions,
                joint,
                self.reference_joint_rad,
                strict=True,
            )
        )

    @property
    def is_valid(self) -> bool:
        return self.state is CalibrationState.VALID

    def validate_for(
        self,
        *,
        model_hash: str,
        board_boot_id: str,
        joint_names: Sequence[str],
        motor_ids: Sequence[int],
    ) -> None:
        """Reject a manifest unless every hardware/model identity matches."""
        if not self.is_valid:
            raise ValueError("calibration manifest is not valid")
        if self.model_hash != model_hash:
            raise ValueError("calibration model_hash does not match active model")
        if self.board_boot_id != board_boot_id:
            raise ValueError("calibration board_boot_id does not match current board")
        if tuple(joint_names) != self.joint_names:
            raise ValueError("calibration joint_names do not match active model")
        if tuple(int(item) for item in motor_ids) != self.motor_ids:
            raise ValueError("calibration motor_ids do not match active mapping")

    def to_dict(self) -> dict[str, Any]:
        return {
            "calibration_id": self.calibration_id,
            "model_hash": self.model_hash,
            "board_boot_id": self.board_boot_id,
            "joint_names": list(self.joint_names),
            "motor_ids": list(self.motor_ids),
            "directions": list(self.directions),
            "zero_offsets_rad": list(self.zero_offsets_rad),
            "reference_joint_rad": list(self.reference_joint_rad),
            "sample_count": self.sample_count,
            "captured_at_utc": self.captured_at_utc,
            "state": self.state.value,
            "metadata": dict(self.metadata),
            "gear_ratio": self.gear_ratio,
        }

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> CalibrationManifest:
        return cls(**dict(value))


@dataclass(frozen=True, slots=True)
class PlanRecord:
    plan_id: str
    model_hash: str
    calibration_id: str
    times_s: tuple[float, ...]
    positions_rad: tuple[tuple[float, ...], ...]
    velocities_rad_s: tuple[tuple[float, ...], ...]
    collision_checked: bool = False
    validated: bool = False

    def __post_init__(self) -> None:
        object.__setattr__(self, "plan_id", _required_text(self.plan_id, name="plan_id"))
        object.__setattr__(self, "model_hash", _required_text(self.model_hash, name="model_hash"))
        object.__setattr__(
            self, "calibration_id", _required_text(self.calibration_id, name="calibration_id")
        )
        times = _finite_vector(self.times_s, name="times_s")
        positions = tuple(
            _finite_vector(row, name="positions_rad row") for row in self.positions_rad
        )
        velocities = tuple(
            _finite_vector(row, name="velocities_rad_s row") for row in self.velocities_rad_s
        )
        if not times or len(times) != len(positions) or len(times) != len(velocities):
            raise ValueError(
                "plan times, positions, and velocities must have the same non-zero length"
            )
        if any(
            current < 0 or (index and current <= times[index - 1])
            for index, current in enumerate(times)
        ):
            raise ValueError("plan times must be non-negative and strictly increasing")
        dof = len(positions[0])
        if dof == 0 or any(len(row) != dof for row in positions + velocities):
            raise ValueError("plan rows must have a consistent non-zero degree of freedom")
        object.__setattr__(self, "times_s", times)
        object.__setattr__(self, "positions_rad", positions)
        object.__setattr__(self, "velocities_rad_s", velocities)

    def validate_for(self, *, model_hash: str, calibration_id: str) -> None:
        if self.model_hash != model_hash:
            raise ValueError("plan model_hash does not match active model")
        if self.calibration_id != calibration_id:
            raise ValueError("plan calibration_id does not match active calibration")
        if not self.validated:
            raise ValueError("plan has not been validated")
        if not self.collision_checked:
            raise ValueError("plan has not passed collision checking")

    def to_dict(self) -> dict[str, Any]:
        return {
            "plan_id": self.plan_id,
            "model_hash": self.model_hash,
            "calibration_id": self.calibration_id,
            "times_s": list(self.times_s),
            "positions_rad": [list(row) for row in self.positions_rad],
            "velocities_rad_s": [list(row) for row in self.velocities_rad_s],
            "collision_checked": self.collision_checked,
            "validated": self.validated,
        }


@dataclass(frozen=True, slots=True)
class JointSnapshot:
    name: str
    motor_id: int
    q_rotor_rad: float
    q_joint_rad: float | None
    dq_joint_rad_s: float
    tau_nm: float
    temperature_c: float
    error: int = 0

    def __post_init__(self) -> None:
        object.__setattr__(self, "name", _required_text(self.name, name="name"))
        if type(self.motor_id) is not int or self.motor_id < 0 or self.motor_id > 14:
            raise ValueError("motor_id must be an integer in [0, 14]")
        for name in ("q_rotor_rad", "dq_joint_rad_s", "tau_nm", "temperature_c"):
            if not math.isfinite(float(getattr(self, name))):
                raise ValueError(f"{name} must be finite")
        if self.q_joint_rad is not None and not math.isfinite(float(self.q_joint_rad)):
            raise ValueError("q_joint_rad must be finite or None")

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "motor_id": self.motor_id,
            "q_rotor": self.q_rotor_rad,
            "q_joint": self.q_joint_rad,
            "dq_joint": self.dq_joint_rad_s,
            "tau": self.tau_nm,
            "temperature_c": self.temperature_c,
            "error": self.error,
        }


@dataclass(frozen=True, slots=True)
class ArmSnapshot:
    sequence: int
    monotonic_ns: int
    state: ControllerState
    model_hash: str
    calibration_id: str | None
    board_boot_id: str
    joints: tuple[JointSnapshot, ...]
    gravity_scale: float = 0.0
    active_plan_id: str | None = None
    fault: str | None = None
    lease_client_id: str | None = None

    @property
    def calibration_valid(self) -> bool:
        return self.calibration_id is not None

    def to_dict(self) -> dict[str, Any]:
        return {
            "sequence": self.sequence,
            "monotonic_ns": self.monotonic_ns,
            "controller_state": self.state.value,
            "model_hash": self.model_hash,
            "calibration_id": self.calibration_id,
            "board_boot_id": self.board_boot_id,
            "joints": [joint.to_dict() for joint in self.joints],
            "gravity_scale": self.gravity_scale,
            "active_plan_id": self.active_plan_id,
            "fault": self.fault,
            "lease": {"client_id": self.lease_client_id} if self.lease_client_id else None,
            "calibration_valid": self.calibration_valid,
        }

    def __getitem__(self, key: str) -> Any:
        return self.to_dict()[key]


def stable_id(value: Mapping[str, Any], *, prefix: str) -> str:
    """Return a deterministic short identity for a manifest/plan payload."""
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode()
    return f"{prefix}-{hashlib.sha256(encoded).hexdigest()[:16]}"
