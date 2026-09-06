from __future__ import annotations

import json
import os
import shutil
from collections.abc import Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from qarm_sim.config import JointMap
from qarm_sim.telemetry import TelemetryError, TelemetrySample


@dataclass(frozen=True)
class ZeroEstimate:
    source_at_reference_rad: NDArray[np.float64]
    rotor_at_reference_rad: NDArray[np.float64]
    source_zero_rad: NDArray[np.float64]
    rotor_zero_rad: NDArray[np.float64]
    reference_joint_rad: NDArray[np.float64]
    position_span_rad: NDArray[np.float64]
    first_sequence: int
    last_sequence: int
    samples: int


def _ordered_sample(
    sample: TelemetrySample, mapping: JointMap, field: str
) -> NDArray[np.float64]:
    by_id = {motor.motor_id: motor for motor in sample.motors}
    result = []
    for motor_id in mapping.motor_ids_by_joint:
        motor = by_id.get(int(motor_id))
        if motor is None:
            raise TelemetryError(f"sample is missing motor ID {int(motor_id)}")
        if not motor.correct:
            raise TelemetryError(
                f"motor ID {int(motor_id)} has invalid feedback"
            )
        if motor.error != 0:
            raise TelemetryError(
                f"motor ID {int(motor_id)} reports error {motor.error}"
            )
        result.append(motor.field(field))
    vector = np.asarray(result, dtype=np.float64)
    if not np.isfinite(vector).all():
        raise TelemetryError(f"non-finite {field} in calibration sample")
    return vector


def estimate_zero(
    samples: Sequence[TelemetrySample],
    mapping: JointMap,
    *,
    maximum_span_rad: float = 0.01,
    reference_joint_rad: NDArray[np.float64] | None = None,
    gear_ratio: float = 6.33,
) -> ZeroEstimate:
    if len(samples) < 20:
        raise ValueError("zero calibration requires at least 20 samples")
    if not np.isfinite(maximum_span_rad) or maximum_span_rad <= 0:
        raise ValueError("maximum_span_rad must be positive and finite")
    reference = (
        np.zeros(len(mapping.joint_names), dtype=np.float64)
        if reference_joint_rad is None
        else np.asarray(reference_joint_rad, dtype=np.float64)
    )
    if reference.shape != (len(mapping.joint_names),):
        raise ValueError(f"reference_joint_rad must contain {len(mapping.joint_names)} values")
    if not np.isfinite(reference).all():
        raise ValueError("reference_joint_rad must be finite")
    if not np.isfinite(gear_ratio) or gear_ratio <= 0:
        raise ValueError("gear_ratio must be positive and finite")
    source = np.stack(
        [
            _ordered_sample(sample, mapping, mapping.angle_field)
            for sample in samples
        ]
    )
    rotor = np.stack(
        [_ordered_sample(sample, mapping, "q_sdk_rad") for sample in samples]
    )
    span = np.ptp(source, axis=0)
    unstable = np.flatnonzero(span > maximum_span_rad)
    if unstable.size:
        details = ", ".join(
            f"{mapping.joint_names[index]}={span[index]:.6f}rad"
            for index in unstable
        )
        raise ValueError(
            "arm moved during zero capture; position spans exceed "
            f"{maximum_span_rad:.6f} rad: {details}"
        )
    source_at_reference = np.median(source, axis=0)
    rotor_at_reference = np.median(rotor, axis=0)
    return ZeroEstimate(
        source_at_reference_rad=source_at_reference,
        rotor_at_reference_rad=rotor_at_reference,
        source_zero_rad=(
            source_at_reference - mapping.direction * reference
        ),
        rotor_zero_rad=(
            rotor_at_reference
            - mapping.direction * gear_ratio * reference
        ),
        reference_joint_rad=reference.copy(),
        position_span_rad=span,
        first_sequence=samples[0].sequence,
        last_sequence=samples[-1].sequence,
        samples=len(samples),
    )


def save_zero_calibration(
    path: Path,
    mapping: JointMap,
    estimate: ZeroEstimate,
    *,
    board_boot_id: str,
) -> Path:
    with path.open(encoding="utf-8") as stream:
        raw = json.load(stream)
    if raw.get("motor_ids_by_joint") != mapping.motor_ids_by_joint.tolist():
        raise ValueError("joint-map motor IDs changed during calibration")
    captured_at = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    backup = path.with_name(f"{path.stem}.prezero-{captured_at}{path.suffix}")
    shutil.copy2(path, backup)
    direction_calibrated = bool(raw.get("direction_calibrated", False))
    raw["zero_offset_rad"] = estimate.source_zero_rad.tolist()
    raw["zero_calibrated"] = True
    raw["calibrated"] = direction_calibrated
    raw["calibration"] = {
        "captured_at_utc": captured_at,
        "board_boot_id": board_boot_id,
        "motor_power_cycle_requires_recapture": True,
        "query_mode": "BRAKE",
        "angle_field": mapping.angle_field,
        "samples": estimate.samples,
        "first_sequence": estimate.first_sequence,
        "last_sequence": estimate.last_sequence,
        "position_span_rad": estimate.position_span_rad.tolist(),
        "source_at_reference_rad": (
            estimate.source_at_reference_rad.tolist()
        ),
        "rotor_at_reference_rad": (
            estimate.rotor_at_reference_rad.tolist()
        ),
        "reference_joint_rad": estimate.reference_joint_rad.tolist(),
        "reference_joint_deg": np.degrees(
            estimate.reference_joint_rad
        ).tolist(),
        "rotor_zero_rad": estimate.rotor_zero_rad.tolist(),
    }
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    try:
        temporary.write_text(
            json.dumps(raw, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()
    return backup
