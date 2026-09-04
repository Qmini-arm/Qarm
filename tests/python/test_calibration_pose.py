import json
from pathlib import Path

import numpy as np
from qarm_sim.calibration_pose import solve_table_supported_pose

ROOT = Path(__file__).resolve().parents[2]


def test_stl_calibration_pose_matches_accepted_manual_setup() -> None:
    solution = solve_table_supported_pose()
    assert np.allclose(
        solution.joint_position_rad,
        [
            0.0,
            1.7480178111,
            0.1548064707,
            -0.0200523360,
            0.0,
            -1.57,
        ],
        atol=2e-8,
    )
    assert abs(solution.arm_link_gap_m) < 1e-8
    assert abs(solution.motor_3_gap_m) < 1e-8
    assert solution.motor_4_axis_vertical_error_deg < 0.1
    assert solution.motor_5_table_clearance_m > 0.08

    configured = json.loads(
        (ROOT / "config/calibration_pose.json").read_text()
    )
    assert np.allclose(
        configured["reference_joint_rad"],
        solution.joint_position_rad,
        atol=2e-8,
    )
