import json

import numpy as np
from qarm_sim.config import JointMap
from qarm_sim.telemetry import TelemetrySample, map_joint_state


def _line() -> str:
    motors = []
    for motor_id in range(6):
        motors.append(
            {
                "id": motor_id,
                "correct": True,
                "q_sdk_rad": motor_id * 6.33,
                "dq_sdk_rad_s": motor_id * 0.633,
                "q_output_rad": motor_id,
                "dq_output_rad_s": motor_id * 0.1,
                "tau_sdk_nm": motor_id * 0.2,
                "tau_ideal_output_nm": motor_id * 0.2 * 6.33,
                "temperature_c": 30 + motor_id,
                "error": 0,
            }
        )
    return json.dumps(
        {
            "type": "sample",
            "monotonic_ns": 100,
            "sequence": 7,
            "motors": motors,
        }
    )


def test_parse_and_map_motor_order() -> None:
    sample = TelemetrySample.from_line(_line())
    assert sample is not None
    mapping = JointMap(
        joint_names=tuple(f"joint_{i}" for i in range(1, 7)),
        motor_ids_by_joint=np.array([0, 1, 2, 3, 4, 5]),
        angle_field="q_output_rad",
        velocity_field="dq_output_rad_s",
        torque_field="tau_ideal_output_nm",
        direction=np.array([1, -1, 1, 1, 1, 1], dtype=float),
        zero_offset_rad=np.array([0, 0.5, 0, 0, 1, 0], dtype=float),
        calibrated=True,
    )
    state = map_joint_state(sample, mapping)
    assert np.allclose(state.position, [0, -0.5, 2, 3, 3, 5])
    assert np.allclose(state.velocity, [0, -0.1, 0.2, 0.3, 0.4, 0.5])
    assert sample.host_receive_monotonic_ns is None


def test_reference_capture_survives_later_direction_change() -> None:
    sample = TelemetrySample.from_line(_line())
    assert sample is not None
    reference = np.array([0.0, 1.7, 0.2, 0.0, 0.0, -1.57])
    source_at_reference = np.arange(6, dtype=float)
    mapping = JointMap(
        joint_names=tuple(f"joint_{i}" for i in range(1, 7)),
        motor_ids_by_joint=np.arange(6),
        angle_field="q_output_rad",
        velocity_field="dq_output_rad_s",
        torque_field="tau_ideal_output_nm",
        direction=np.array([-1, 1, -1, 1, -1, 1], dtype=float),
        zero_offset_rad=np.zeros(6),
        calibrated=False,
        zero_calibrated=True,
        direction_calibrated=True,
        calibration_reference_joint_rad=reference,
        source_at_reference_rad=source_at_reference,
    )
    state = map_joint_state(sample, mapping)
    assert np.allclose(state.position, reference)
