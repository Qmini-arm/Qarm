"""Offline tests for the Qarm control plane.

No test in this module opens a serial device or invokes an installed motor
binary. Hardware behavior is exercised with an injected fake process factory.
"""

from __future__ import annotations

import csv
import io
import json
import sys
import threading
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

# The server is intentionally runnable as a standalone script, so keep tests
# independent of an installed Python package.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from qarm_control_server import (
    CALIBRATION_POSE,
    CONFIG,
    URDF,
    ActionError,
    ControlState,
    create_server,
    validate_program,
)


class FakeProcess:
    def __init__(self, **_kwargs: object) -> None:
        self.stdout = io.StringIO("")
        self._returncode: int | None = None
        self.terminated = False

    def poll(self) -> int | None:
        return self._returncode

    def terminate(self) -> None:
        self.terminated = True
        self._returncode = -15

    def kill(self) -> None:
        self._returncode = -9

    def wait(self, timeout: float | None = None) -> int:
        del timeout
        return self._returncode or 0


def _request(server, path: str, method: str = "GET", body: dict | None = None):
    url = f"http://127.0.0.1:{server.server_port}{path}"
    data = None if body is None else json.dumps(body).encode()
    request = urllib.request.Request(
        url, data=data, method=method, headers={"Content-Type": "application/json"}
    )
    try:
        with urllib.request.urlopen(request, timeout=2) as response:
            return response.status, json.loads(response.read())
    except urllib.error.HTTPError as error:
        return error.code, json.loads(error.read())


@pytest.fixture()
def sim_server():
    state = ControlState(hardware=False)
    server = create_server(state, host="127.0.0.1", port=0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield server, state
    server.shutdown()
    server.server_close()


def test_health_and_simulation_state(sim_server):
    server, _ = sim_server
    status, health = _request(server, "/api/health")
    assert status == 200
    assert health == {"ok": True, "service": "qarm-control", "mode": "simulation"}
    _, payload = _request(server, "/api/status")
    assert payload["connected"] is True
    assert payload["enabled"] is False
    assert payload["capabilities"]["movej"] is True
    assert payload["dof"] == 4
    assert payload["joint_names"] == ["joint_1", "joint_2", "joint_3", "joint_4"]
    assert [joint["id"] for joint in payload["joints"]] == [0, 1, 2, 3]
    assert payload["calibrated"] is False
    assert payload["capabilities"]["return_home"] is False
    _, config = _request(server, "/api/config")
    assert config["joint_limits_rad"] == [
        [joint["min"], joint["max"]] for joint in payload["joints"]
    ]
    assert config["calibration_pose_rad"] is None


def test_enable_then_gravity_and_movej(sim_server):
    server, _ = sim_server
    status, payload = _request(server, "/api/gravity", "POST", {"enabled": True})
    assert status == 409 and payload["status"]["gravity"] is False
    assert _request(server, "/api/enable", "POST", {"enabled": True})[0] == 200
    assert _request(server, "/api/gravity", "POST", {"enabled": True})[1]["gravity"] is True
    angles = [0.1, 0.2, 0.3, 0.4]
    status, payload = _request(
        server, "/api/movej", "POST", {"joints": angles, "speed": 0.2, "acceleration": 0.4}
    )
    assert status == 200
    assert [joint["angle"] for joint in payload["joints"]] == angles


def test_simulation_movej_rejects_bad_numbers(sim_server):
    server, _ = sim_server
    _request(server, "/api/enable", "POST", {"enabled": True})
    assert _request(server, "/api/movej", "POST", {"joints": [0] * 6})[0] == 400
    assert _request(server, "/api/movej", "POST", {"joints": [0, 0, 0, float("nan")]})[0] == 400
    assert _request(server, "/api/movej", "POST", {"joints": [0] * 4, "speed": 0})[0] == 400
    assert _request(server, "/api/movej", "POST", {"joints": [0, 0, 0, 2.1]})[0] == 400


def test_estop_latches_and_clear_requires_reenable(sim_server):
    server, _ = sim_server
    _request(server, "/api/enable", "POST", {"enabled": True})
    _request(server, "/api/gravity", "POST", {"enabled": True})
    _, stopped = _request(server, "/api/estop", "POST", {})
    assert stopped["estop"] and not stopped["enabled"] and not stopped["gravity"]
    assert _request(server, "/api/movej", "POST", {"joints": [0] * 4})[0] == 409
    _, cleared = _request(server, "/api/clear-estop", "POST", {})
    assert cleared["estop"] is False and cleared["enabled"] is False


def write_trajectory(path: Path, names: list[str], values: list[float]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "time_s",
                *(f"{name}_position_rad" for name in names),
                *(f"{name}_velocity_rad_s" for name in names),
            ]
        )
        writer.writerow([0.0, *values, *([0.0] * len(names))])


def test_return_home_requires_existing_collision_checked_plan(sim_server, tmp_path: Path):
    server, _ = sim_server
    _request(server, "/api/enable", "POST", {"enabled": True})
    assert (
        _request(
            server,
            "/api/return-home",
            "POST",
            {
                "trajectory_path": str(tmp_path / "missing.csv"),
                "confirm_collision_checked_plan": True,
            },
        )[0]
        == 400
    )
    trajectory = tmp_path / "calibration_home.csv"
    write_trajectory(trajectory, [f"joint_{index}" for index in range(1, 5)], [0.0] * 4)
    assert (
        _request(server, "/api/return-home", "POST", {"trajectory_path": str(trajectory)})[0] == 400
    )
    status, payload = _request(
        server,
        "/api/return-home",
        "POST",
        {"trajectory_path": str(trajectory), "confirm_collision_checked_plan": True},
    )
    assert status == 409
    assert "标定" in payload["error"]


def test_old_six_axis_trajectory_is_rejected_before_launch(tmp_path: Path):
    calls = []
    state = ControlState(hardware=True, popen=lambda *args, **kwargs: calls.append(args))
    trajectory = tmp_path / "old-six-axis.csv"
    write_trajectory(trajectory, [f"joint_{index}" for index in range(1, 7)], [0.0] * 6)
    with pytest.raises(ActionError, match="active joint names") as error:
        state.return_home(str(trajectory), True)
    assert error.value.status == 400
    assert not calls


def test_hardware_never_claims_enable_or_gravity_without_adapter():
    state = ControlState(hardware=True, popen=lambda *args, **kwargs: FakeProcess(**kwargs))
    assert state.snapshot()["connected"] is False
    with pytest.raises(Exception) as error:
        state.set_enabled(True)
    assert error.value.status == 409
    state.connected = True
    with pytest.raises(Exception) as error:
        state.set_enabled(True)
    assert error.value.status == 501 and state.enabled is False
    state.enabled = True  # emulate an external controller adapter for this guard test
    with pytest.raises(Exception) as error:
        state.set_gravity(True)
    assert error.value.status == 501 and state.gravity is False


def test_hardware_connect_waits_for_feedback_and_uses_reader_factory():
    calls: list[list[str]] = []

    def fake_popen(command, **kwargs):
        calls.append(command)
        return FakeProcess(**kwargs)

    state = ControlState(hardware=True, popen=fake_popen)
    payload = state.connect()
    assert payload["connected"] is False
    assert payload["lifecycle"] == "connecting"
    assert calls and calls[0][0] == state.reader_path
    assert "--acknowledge-state-change" in calls[0]
    assert calls[0][calls[0].index("--ids") + 1] == "0,1,2,3"


def test_reader_sample_updates_all_joints_and_marks_feedback_online():
    process = FakeProcess()
    state = ControlState(hardware=True, popen=lambda *_args, **_kwargs: process)
    state.reader = process
    motors = [
        {
            "id": i,
            "q_output_rad": i / 10,
            "dq_output_rad_s": i / 100,
            "tau_ideal_output_nm": i / 20,
            "temperature_c": 31 + i,
            "error": 0,
        }
        for i in range(4)
    ]
    process.stdout = io.StringIO(json.dumps({"type": "sample", "motors": motors}) + "\n")
    state._read_reader()
    assert state.connected is True
    assert state.lifecycle == "connected_read_only"
    assert state.joints[3]["angle"] == pytest.approx(0.3)


def test_hardware_return_home_is_async_and_not_reported_complete(tmp_path: Path):
    trajectory = tmp_path / "calibration_home.csv"
    pose = json.loads(CALIBRATION_POSE.read_text(encoding="utf-8"))
    pose["validated"] = True
    pose_path = tmp_path / "calibration_pose.json"
    pose_path.write_text(json.dumps(pose), encoding="utf-8")
    mapping = json.loads(CONFIG.read_text(encoding="utf-8"))
    for field in ("calibrated", "zero_calibrated", "direction_calibrated"):
        mapping[field] = True
    mapping_path = tmp_path / "joint_map.json"
    mapping_path.write_text(json.dumps(mapping), encoding="utf-8")
    write_trajectory(trajectory, pose["joint_names"], pose["reference_joint_rad"])
    process = FakeProcess()
    calls = []

    def fake_popen(command, **kwargs):
        calls.append(command)
        return process

    state = ControlState(
        hardware=True, popen=fake_popen, config_path=mapping_path, calibration_pose_path=pose_path
    )
    state.connected, state.enabled = True, True
    payload = state.return_home(str(trajectory), True)
    assert payload["lifecycle"] == "moving_to_calibration"
    assert payload["last_action"] == "return_home_started"
    assert calls[0][0] == state.return_home_path


def test_six_axis_mapping_rejected(tmp_path: Path):
    mapping = json.loads(CONFIG.read_text(encoding="utf-8"))
    mapping["joint_names"] += ["joint_5", "joint_6"]
    path = tmp_path / "legacy-map.json"
    path.write_text(json.dumps(mapping), encoding="utf-8")
    with pytest.raises(ValueError, match="active URDF chain"):
        ControlState(config_path=path)


def test_program_validates_names_width_limits_and_malformed_nodes(sim_server):
    server, state = sim_server
    program = {
        "version": 1,
        "joint_names": state.joint_names,
        "nodes": [
            {"type": "start"},
            {"type": "movej", "joints": [0.1, 0.2, 0.3, 0.4]},
            {"type": "wait", "duration_s": 1},
            {"type": "end"},
        ],
    }
    assert _request(server, "/api/program/validate", "POST", program)[0] == 200
    program["nodes"][1]["joints"] = [0.0] * 6
    assert _request(server, "/api/program/validate", "POST", program)[0] == 422
    program["nodes"][1]["joints"] = [0, 0, 0, 2.1]
    assert _request(server, "/api/program/validate", "POST", program)[0] == 422
    program["nodes"][1]["joints"] = [0.0] * 4
    program["joint_names"] = ["joint_4", "joint_3", "joint_2", "joint_1"]
    assert _request(server, "/api/program/validate", "POST", program)[0] == 422
    assert validate_program({"version": 1, "nodes": [None, 1]}, state.joints)


def test_reader_ignores_legacy_six_motor_packets():
    process = FakeProcess()
    state = ControlState(hardware=True)
    state.reader = process
    process.stdout = io.StringIO(
        json.dumps({"type": "sample", "motors": [{"id": index} for index in range(6)]}) + "\n"
    )
    state._read_reader()
    assert not state.connected


def test_calibrated_reader_keeps_reference_anchor_after_direction_change(tmp_path: Path):
    mapping = json.loads(CONFIG.read_text(encoding="utf-8"))
    mapping.update(calibrated=True, zero_calibrated=True, direction_calibrated=True)
    mapping["direction"] = [-1, 1, 1, 1]
    mapping["zero_offset_rad"] = [1.3, 0, 0, 0]
    mapping["calibration"] = {
        "reference_joint_rad": [0.7, 0, 0, 0],
        "source_at_reference_rad": [2.0, 0, 0, 0],
    }
    path = tmp_path / "joint_map.json"
    path.write_text(json.dumps(mapping), encoding="utf-8")
    state = ControlState(hardware=True, config_path=path)
    motors = [
        {
            "id": index,
            "q_output_rad": 2.2 if index == 0 else 0,
            "dq_output_rad_s": 0.1,
            "tau_ideal_output_nm": 0.2,
            "temperature_c": 30,
        }
        for index in range(4)
    ]
    process = FakeProcess()
    process.stdout = io.StringIO(json.dumps({"type": "sample", "motors": motors}) + "\n")
    state.reader = process
    state._read_reader()
    assert state.joints[0]["angle"] == pytest.approx(0.5)
    assert state.joints[0]["velocity"] == pytest.approx(-0.1)
    assert state.joints[0]["torque"] == pytest.approx(-0.2)


@pytest.mark.parametrize("values", [[0.0] * 6, [0.0, 0.0, 0.0, float("nan")], None])
def test_calibration_anchors_reject_wrong_width_or_nonfinite(tmp_path: Path, values):
    mapping = json.loads(CONFIG.read_text(encoding="utf-8"))
    mapping["calibration"] = {"reference_joint_rad": [0.0] * 4, "source_at_reference_rad": values}
    path = tmp_path / "joint_map.json"
    path.write_text(json.dumps(mapping), encoding="utf-8")
    with pytest.raises(ValueError, match="source_at_reference_rad"):
        ControlState(config_path=path)


@pytest.mark.parametrize("motor_id", [-1, 15])
def test_mapping_rejects_motor_ids_outside_protocol(tmp_path: Path, motor_id: int):
    mapping = json.loads(CONFIG.read_text(encoding="utf-8"))
    mapping["motor_ids_by_joint"][0] = motor_id
    path = tmp_path / "joint_map.json"
    path.write_text(json.dumps(mapping), encoding="utf-8")
    with pytest.raises(ValueError, match="motor ID"):
        ControlState(config_path=path)


def test_model_intersects_soft_and_hard_limits(tmp_path: Path):
    tree = ET.parse(URDF)
    joint = tree.getroot().find("joint[@name='joint_4']")
    joint.find("safety_controller").set("soft_lower_limit", "-10")
    joint.find("safety_controller").set("soft_upper_limit", "10")
    path = tmp_path / "arm.urdf"
    tree.write(path, encoding="utf-8")
    state = ControlState(urdf_path=path)
    assert state.joints[3]["min"] == pytest.approx(-2.094395102)
    assert state.joints[3]["max"] == pytest.approx(2.094395102)
