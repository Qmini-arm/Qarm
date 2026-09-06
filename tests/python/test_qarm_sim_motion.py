from __future__ import annotations

import csv
import json

import mujoco
import numpy as np
import pytest
from qarm_sim.cli import _parser, _remote_command, main
from qarm_sim.config import DEFAULT_JOINT_MAP
from qarm_sim.model import build_scene, set_mirrored_state
from qarm_sim.motion import OfflineMotion, write_joint_trajectory_csv
from qmini_arm_motion.cli import main as legacy_main
from qmini_arm_motion.planner import TimedTrajectory


@pytest.mark.parametrize("entrypoint, error_code", [(main, 2), (legacy_main, 1)])
def test_fk_rejects_stale_six_axis_vector(entrypoint, error_code, capsys) -> None:
    assert entrypoint(["fk", "--q-deg", "0", "0", "0", "0", "0", "0"]) == error_code
    assert "requires 4 joint angles" in capsys.readouterr().err


def test_remote_reader_uses_only_configured_motor_ids(tmp_path) -> None:
    raw = json.loads(DEFAULT_JOINT_MAP.read_text())
    raw["motor_ids_by_joint"] = [3, 7, 9, 12]
    path = tmp_path / "joint_map.json"
    path.write_text(json.dumps(raw))
    args = _parser().parse_args(["inspect-stream", "--joint-map", str(path)])
    assert "--ids 3,7,9,12 --mode brake" in _remote_command(args)[-1]


def test_joint_csv_rejects_legacy_velocity_width_before_writing(tmp_path) -> None:
    trajectory = TimedTrajectory(
        times_s=np.array([0.0, 0.01]),
        positions_rad=np.zeros((2, 4)),
        velocities_rad_s=np.zeros((2, 6)),
    )
    output = tmp_path / "bad.csv"
    with pytest.raises(ValueError, match="positions and velocities must have shape"):
        write_joint_trajectory_csv(
            trajectory, tuple(f"joint_{index}" for index in range(1, 5)), output
        )
    assert not output.exists()


def test_unified_fk_command_uses_offline_motion_stack(capsys) -> None:
    result = main(["fk", "--q-deg", "0", "0", "0", "0"])

    assert result == 0
    report = json.loads(capsys.readouterr().out)
    assert report["execution"] == "offline_only"
    assert report["hardware_io_performed"] is False
    assert report["frame"] == "base_link"
    assert report["joint_names"] == [f"joint_{index}" for index in range(1, 5)]
    assert np.allclose(report["joint_position_rad"], 0.0)
    assert np.allclose(
        report["tool0_position_m"],
        [0.742450235, -0.004895224, -0.006285450],
        atol=1e-8,
    )
    assert report["self_collision"] == []


def test_unified_plan_home_exports_offline_joint_trajectory(tmp_path, capsys) -> None:
    output = tmp_path / "home.csv"
    result = main(
        [
            "plan-home",
            "--start-deg",
            "10",
            "5",
            "10",
            "5",
            "--output",
            str(output),
        ]
    )

    assert result == 0
    report = json.loads(capsys.readouterr().out)
    assert report["execution"] == "offline_only"
    assert report["hardware_io_performed"] is False
    assert report["goal"] == "desk-supported calibration pose"
    assert np.allclose(
        report["goal_joint_position_rad"],
        [0.0, 1.7480178110996762, 0.1548064706928587, 0.0],
    )
    assert report["path_kind"] == "joint_direct"
    assert report["mujoco_simulation"]["passed"] is True
    assert report["mujoco_simulation"]["hard_limit_violations"] == 0
    assert report["mujoco_simulation"]["final_floor_contact"] is True
    assert report["control_period_s"] == 0.01
    assert report["trajectory_csv"] == str(output.resolve())

    with output.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == report["samples"]
    assert np.isclose(float(rows[0]["joint_1_position_rad"]), np.radians(10.0), atol=1e-9)
    assert np.isclose(float(rows[1]["time_s"]) - float(rows[0]["time_s"]), 0.01)
    expected_goal = [
        0.0,
        1.7480178110996762,
        0.1548064706928587,
        0.0,
    ]
    assert np.allclose(
        [float(rows[-1][f"joint_{index}_position_rad"]) for index in range(1, 5)],
        expected_goal,
    )
    assert all(float(rows[-1][f"joint_{index}_velocity_rad_s"]) == 0.0 for index in range(1, 5))


def test_unified_cartesian_plan_command_reaches_target(capsys) -> None:
    motion = OfflineMotion.load()
    known_goal = np.array([0.20, 0.10, 0.30, 0.10])
    target = motion.model.fk(known_goal)[:3, 3]

    result = main(["plan", "--target", *(str(value) for value in target)])

    assert result == 0
    report = json.loads(capsys.readouterr().out)
    assert report["execution"] == "offline_only"
    assert report["hardware_io_performed"] is False
    assert report["ik_status"] == "converged"
    assert report["position_error_m"] <= 0.001
    assert report["trajectory_csv"] is None


def test_motion_fk_matches_mujoco_in_base_link_frame() -> None:
    motion = OfflineMotion.load()
    scene = build_scene()
    base_id = mujoco.mj_name2id(scene.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    configurations = np.array(
        [
            np.zeros(4),
            [0.2, -0.3, 0.4, -0.25],
            [-0.4, 0.2, -0.2, 0.3],
        ]
    )

    for position in configurations:
        set_mirrored_state(scene, position)
        base_rotation = scene.data.xmat[base_id].reshape(3, 3)
        tool_rotation = scene.data.site_xmat[scene.tool_site_id].reshape(3, 3)
        mujoco_position = base_rotation.T @ (
            scene.data.site_xpos[scene.tool_site_id] - scene.data.xpos[base_id]
        )
        mujoco_rotation = base_rotation.T @ tool_rotation
        analytic = motion.model.fk(position)

        # MuJoCo's URDF normalization serializes poses at slightly lower precision.
        assert np.allclose(analytic[:3, 3], mujoco_position, rtol=0.0, atol=1e-6)
        assert np.allclose(analytic[:3, :3], mujoco_rotation, rtol=0.0, atol=3e-6)


def test_unified_plan_urdf_zero_is_separate_from_power_down_pose(capsys) -> None:
    result = main(
        [
            "plan-urdf-zero",
            "--start-deg",
            "10",
            "5",
            "10",
            "5",
        ]
    )

    assert result == 0
    report = json.loads(capsys.readouterr().out)
    assert report["goal"] == "URDF zero configuration"
    assert np.allclose(report["goal_joint_position_rad"], 0.0)
    assert report["mujoco_simulation"]["passed"] is True
