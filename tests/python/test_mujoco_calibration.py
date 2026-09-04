import json

import numpy as np
import pytest
from qarm_sim.calibration import estimate_zero, save_zero_calibration
from qarm_sim.config import JointMap
from qarm_sim.telemetry import TelemetrySample


def _mapping() -> JointMap:
    return JointMap(
        joint_names=tuple(f"joint_{i}" for i in range(1, 7)),
        motor_ids_by_joint=np.arange(6, dtype=np.int64),
        angle_field="q_output_rad",
        velocity_field="dq_output_rad_s",
        torque_field="tau_ideal_output_nm",
        direction=np.ones(6),
        zero_offset_rad=np.zeros(6),
        calibrated=False,
    )


def _sample(sequence: int, shift: float = 0.0) -> TelemetrySample:
    motors = []
    for motor_id in range(6):
        q = 0.1 * (motor_id + 1) + shift
        motors.append(
            {
                "id": motor_id,
                "correct": True,
                "q_sdk_rad": q * 6.33,
                "dq_sdk_rad_s": 0.0,
                "q_output_rad": q,
                "dq_output_rad_s": 0.0,
                "tau_sdk_nm": 0.0,
                "tau_ideal_output_nm": 0.0,
                "temperature_c": 25,
                "error": 0,
            }
        )
    result = TelemetrySample.from_line(
        json.dumps(
            {
                "type": "sample",
                "monotonic_ns": sequence * 10_000_000,
                "sequence": sequence,
                "motors": motors,
            }
        )
    )
    assert result is not None
    return result


def test_estimate_and_atomically_save_zero(tmp_path) -> None:
    mapping = _mapping()
    samples = [_sample(i, (i % 3 - 1) * 1e-4) for i in range(30)]
    estimate = estimate_zero(samples, mapping, maximum_span_rad=0.001)
    assert np.allclose(estimate.source_zero_rad, np.arange(1, 7) * 0.1)
    assert np.all(estimate.position_span_rad <= 0.0002 + 1e-12)

    path = tmp_path / "joint_map.json"
    path.write_text(
        json.dumps(
            {
                "joint_names": list(mapping.joint_names),
                "motor_ids_by_joint": mapping.motor_ids_by_joint.tolist(),
                "angle_field": mapping.angle_field,
                "velocity_field": mapping.velocity_field,
                "torque_field": mapping.torque_field,
                "direction": mapping.direction.tolist(),
                "zero_offset_rad": mapping.zero_offset_rad.tolist(),
                "zero_calibrated": False,
                "direction_calibrated": False,
                "calibrated": False,
            }
        )
    )
    backup = save_zero_calibration(
        path, mapping, estimate, board_boot_id="test-boot-id"
    )
    saved = json.loads(path.read_text())
    assert backup.is_file()
    assert saved["zero_calibrated"] is True
    assert saved["direction_calibrated"] is False
    assert saved["calibrated"] is False
    assert saved["calibration"]["board_boot_id"] == "test-boot-id"
    assert np.allclose(saved["zero_offset_rad"], np.arange(1, 7) * 0.1)


def test_zero_capture_rejects_motion() -> None:
    mapping = _mapping()
    samples = [_sample(i, 0.0) for i in range(20)]
    samples[-1] = _sample(19, 0.02)
    with pytest.raises(ValueError, match="arm moved"):
        estimate_zero(samples, mapping, maximum_span_rad=0.01)


def test_reference_pose_is_removed_from_encoder_zero() -> None:
    mapping = _mapping()
    reference = np.array([0.0, -1.7, -0.2, 0.0, 0.0, -1.57])
    samples = [_sample(i) for i in range(20)]
    estimate = estimate_zero(
        samples,
        mapping,
        maximum_span_rad=0.001,
        reference_joint_rad=reference,
        gear_ratio=6.33,
    )
    measured_output = np.arange(1, 7) * 0.1
    assert np.allclose(
        estimate.source_zero_rad, measured_output - reference
    )
    assert np.allclose(estimate.source_at_reference_rad, measured_output)
    assert np.allclose(
        estimate.rotor_zero_rad,
        measured_output * 6.33 - 6.33 * reference,
    )
