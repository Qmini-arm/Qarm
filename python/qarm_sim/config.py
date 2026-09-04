from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MOTOR_CONFIG = ROOT / "config/m8010.json"
DEFAULT_JOINT_MAP = ROOT / "config/joint_map.json"
DEFAULT_XACRO = ROOT / "description/qmini_arm.urdf.xacro"


def _load_json(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"expected an object in {path}")
    return value


def _vector(
    value: Any,
    *,
    name: str,
    size: int,
    dtype: type[np.floating] | type[np.integer] = np.float64,
) -> NDArray:
    result = np.asarray(value, dtype=dtype)
    if result.shape != (size,):
        raise ValueError(f"{name} must contain {size} values, got {result.shape}")
    return result


@dataclass(frozen=True)
class MotorParameters:
    model: str
    gear_ratio: float
    peak_torque_nm: float
    maximum_speed_rad_s: float
    rs485_baud: int
    encoder_bits: int
    source: str

    @classmethod
    def load(cls, path: Path = DEFAULT_MOTOR_CONFIG) -> MotorParameters:
        raw = _load_json(path)
        result = cls(
            model=str(raw["model"]),
            gear_ratio=float(raw["gear_ratio"]),
            peak_torque_nm=float(raw["peak_torque_nm"]),
            maximum_speed_rad_s=float(raw["maximum_speed_rad_s_at_24v"]),
            rs485_baud=int(raw["rs485_baud"]),
            encoder_bits=int(raw["motor_encoder_bits"]),
            source=str(raw["source"]),
        )
        if result.gear_ratio <= 0 or result.peak_torque_nm <= 0:
            raise ValueError("motor gear ratio and peak torque must be positive")
        return result


@dataclass(frozen=True)
class JointMap:
    joint_names: tuple[str, ...]
    motor_ids_by_joint: NDArray[np.int64]
    angle_field: str
    velocity_field: str
    torque_field: str
    direction: NDArray[np.float64]
    zero_offset_rad: NDArray[np.float64]
    calibrated: bool
    zero_calibrated: bool = False
    direction_calibrated: bool = False
    calibration_board_boot_id: str | None = None
    calibration_reference_joint_rad: NDArray[np.float64] | None = None
    source_at_reference_rad: NDArray[np.float64] | None = None

    @classmethod
    def load(cls, path: Path = DEFAULT_JOINT_MAP) -> JointMap:
        raw = _load_json(path)
        names = tuple(str(value) for value in raw["joint_names"])
        size = len(names)
        if size != 6 or len(set(names)) != size:
            raise ValueError("joint map must define six unique joints")
        ids = _vector(
            raw["motor_ids_by_joint"],
            name="motor_ids_by_joint",
            size=size,
            dtype=np.int64,
        )
        if len(set(int(value) for value in ids)) != size:
            raise ValueError("motor_ids_by_joint must be unique")
        direction = _vector(raw["direction"], name="direction", size=size)
        if not np.all(np.isin(direction, (-1.0, 1.0))):
            raise ValueError("direction entries must be +1 or -1")
        zero = _vector(
            raw["zero_offset_rad"], name="zero_offset_rad", size=size
        )
        if not np.isfinite(zero).all():
            raise ValueError("zero offsets must be finite")
        calibration = raw.get("calibration")
        has_reference = (
            isinstance(calibration, dict)
            and calibration.get("reference_joint_rad") is not None
            and calibration.get("source_at_reference_rad") is not None
        )
        reference = (
            _vector(
                calibration["reference_joint_rad"],
                name="calibration.reference_joint_rad",
                size=size,
            )
            if has_reference
            else None
        )
        source_at_reference = (
            _vector(
                calibration["source_at_reference_rad"],
                name="calibration.source_at_reference_rad",
                size=size,
            )
            if has_reference
            else None
        )
        return cls(
            joint_names=names,
            motor_ids_by_joint=ids,
            angle_field=str(raw["angle_field"]),
            velocity_field=str(raw["velocity_field"]),
            torque_field=str(raw["torque_field"]),
            direction=direction,
            zero_offset_rad=zero,
            calibrated=bool(raw.get("calibrated", False)),
            zero_calibrated=bool(raw.get("zero_calibrated", False)),
            direction_calibrated=bool(
                raw.get("direction_calibrated", False)
            ),
            calibration_board_boot_id=(
                str(calibration["board_boot_id"])
                if isinstance(calibration, dict)
                and calibration.get("board_boot_id")
                else None
            ),
            calibration_reference_joint_rad=reference,
            source_at_reference_rad=source_at_reference,
        )
