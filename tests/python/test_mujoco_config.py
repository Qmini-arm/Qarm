import json
from pathlib import Path

import numpy as np
from qarm_sim.config import JointMap, MotorParameters

ROOT = Path(__file__).resolve().parents[2]


def test_official_motor_parameters_are_loaded() -> None:
    motor = MotorParameters.load(ROOT / "config/m8010.json")
    assert motor.model == "GO-M8010-6"
    assert motor.gear_ratio == 6.33
    assert motor.peak_torque_nm == 23.7
    assert motor.rs485_baud == 4_000_000


def test_joint_map_has_operator_verified_mapping() -> None:
    mapping = JointMap.load(ROOT / "config/joint_map.json")
    assert mapping.joint_names == tuple(f"joint_{i}" for i in range(1, 7))
    assert mapping.motor_ids_by_joint.tolist() == [0, 1, 2, 3, 4, 5]
    assert np.all(mapping.direction == 1)
    assert mapping.zero_calibrated
    assert mapping.direction_calibrated
    assert mapping.calibrated


def test_gravity_deploy_config_matches_session_calibration() -> None:
    mapping_raw = json.loads((ROOT / "config/joint_map.json").read_text())
    values = {}
    for raw_line in (ROOT / "config/gravity_comp.conf").read_text().splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        key, value = line.split("=", 1)
        values[key] = value

    def floats(key: str) -> list[float]:
        return [float(value) for value in values[key].split(",")]

    assert int(values["schema_version"]) == 2
    assert float(values["max_compensation_scale"]) == 1.0
    assert np.allclose(
        floats("rotor_torque_caps_nm"),
        [0.03, 2.00, 0.90, 0.08, 0.03, 0.03],
    )
    assert np.allclose(
        floats("joint_speed_trip_rad_s"),
        [0.80, 0.80, 0.80, 1.20, 2.00, 2.50],
    )
    assert np.allclose(
        floats("joint_speed_hard_trip_rad_s"),
        [1.50, 1.50, 1.50, 2.40, 4.00, 5.00],
    )
    assert np.allclose(
        floats("hard_lower_rad"),
        [-3.141592654, -1.75, -2.62, -2.094395102, -2.094395102, -1.57],
    )
    assert np.allclose(
        floats("hard_upper_rad"),
        [3.141592654, 1.75, 2.62, 2.094395102, 2.094395102, 1.57],
    )
    assert int(values["joint_speed_trip_consecutive_cycles"]) == 3
    assert float(values["rotor_torque_slew_nm_per_cycle"]) == 2.0 / 256.0
    assert float(values["rotor_feedback_torque_trip_nm"]) == 3.0
    assert values["expected_board_boot_id"] == mapping_raw["calibration"][
        "board_boot_id"
    ]
    assert [int(value) for value in values["motor_ids"].split(",")] == (
        mapping_raw["motor_ids_by_joint"]
    )
    assert [int(value) for value in values["directions"].split(",")] == (
        mapping_raw["direction"]
    )
    assert np.allclose(
        floats("rotor_at_reference_rad"),
        mapping_raw["calibration"]["rotor_at_reference_rad"],
    )
    assert np.allclose(
        floats("reference_joint_rad"),
        mapping_raw["calibration"]["reference_joint_rad"],
    )
