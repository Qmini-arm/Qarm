"""Offline tests for the Qarm control plane.

No test in this module opens a serial device or invokes an installed motor
binary. Hardware behavior is exercised with an injected fake process factory.
"""

from __future__ import annotations

import io
import json
import sys
import threading
import urllib.error
import urllib.request
from pathlib import Path

import pytest

# The server is intentionally runnable as a standalone script, so keep tests
# independent of an installed Python package.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from qarm_control_server import ControlState, create_server


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


def test_enable_then_gravity_and_movej(sim_server):
    server, _ = sim_server
    status, payload = _request(server, "/api/gravity", "POST", {"enabled": True})
    assert status == 409 and payload["status"]["gravity"] is False
    assert _request(server, "/api/enable", "POST", {"enabled": True})[0] == 200
    assert _request(server, "/api/gravity", "POST", {"enabled": True})[1]["gravity"] is True
    angles = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    status, payload = _request(
        server, "/api/movej", "POST", {"joints": angles, "speed": 0.2, "acceleration": 0.4}
    )
    assert status == 200
    assert [joint["angle"] for joint in payload["joints"]] == angles


def test_simulation_movej_rejects_bad_numbers(sim_server):
    server, _ = sim_server
    _request(server, "/api/enable", "POST", {"enabled": True})
    assert _request(server, "/api/movej", "POST", {"joints": [0] * 5})[0] == 400
    assert (
        _request(server, "/api/movej", "POST", {"joints": [0, 0, 0, 0, 0, float("nan")]})[0] == 400
    )
    assert _request(server, "/api/movej", "POST", {"joints": [0] * 6, "speed": 0})[0] == 400


def test_estop_latches_and_clear_requires_reenable(sim_server):
    server, _ = sim_server
    _request(server, "/api/enable", "POST", {"enabled": True})
    _request(server, "/api/gravity", "POST", {"enabled": True})
    _, stopped = _request(server, "/api/estop", "POST", {})
    assert stopped["estop"] and not stopped["enabled"] and not stopped["gravity"]
    assert _request(server, "/api/movej", "POST", {"joints": [0] * 6})[0] == 409
    _, cleared = _request(server, "/api/clear-estop", "POST", {})
    assert cleared["estop"] is False and cleared["enabled"] is False


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
    trajectory.write_text("0,0,0,0,0,0\n", encoding="utf-8")
    assert (
        _request(server, "/api/return-home", "POST", {"trajectory_path": str(trajectory)})[0] == 400
    )
    status, payload = _request(
        server,
        "/api/return-home",
        "POST",
        {"trajectory_path": str(trajectory), "confirm_collision_checked_plan": True},
    )
    assert status == 200 and payload["lifecycle"] == "holding"
    assert [joint["angle"] for joint in payload["joints"]] == [
        0.0,
        1.748017811,
        0.154806471,
        -0.020052336,
        0.0,
        -1.57,
    ]


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
        for i in range(6)
    ]
    process.stdout = io.StringIO(json.dumps({"type": "sample", "motors": motors}) + "\n")
    state._read_reader()
    assert state.connected is True
    assert state.lifecycle == "connected_read_only"
    assert state.joints[3]["angle"] == pytest.approx(0.3)


def test_hardware_return_home_is_async_and_not_reported_complete(tmp_path: Path):
    trajectory = tmp_path / "calibration_home.csv"
    trajectory.write_text("0,0,0,0,0,0\n", encoding="utf-8")
    process = FakeProcess()
    calls = []

    def fake_popen(command, **kwargs):
        calls.append(command)
        return process

    state = ControlState(hardware=True, popen=fake_popen)
    state.connected, state.enabled = True, True
    payload = state.return_home(str(trajectory), True)
    assert payload["lifecycle"] == "moving_to_calibration"
    assert payload["last_action"] == "return_home_started"
    assert calls[0][0] == state.return_home_path
