"""Control-domain safety tests; no test opens a serial device."""

from __future__ import annotations

import copy
from itertools import count

import pytest

from qarm_control import FakeArmBackend


@pytest.fixture
def backend():
    return FakeArmBackend(
        model_hash="model-v1",
        joint_limits_rad=[[-1.0, 1.0]] * 4,
        collision_validator=lambda path: all(row[0] < 0.9 for row in path),
    )


@pytest.fixture
def send(backend):
    sequence = count()

    def command(name, payload=None, *, client_id="operator", request_id=None):
        return backend.command(
            name, payload, client_id=client_id,
            request_id=request_id or f"request-{next(sequence)}",
        )

    return command


def ready(send):
    assert send("connection.connect")["accepted"]
    assert send("lease.acquire")["accepted"]
    capture = send("zero.capture", {"reference_joint_rad": [0.0] * 4})
    assert capture["accepted"]
    calibration_id = capture["result"]["calibration_candidate"]["calibration_id"]
    committed = send("zero.commit", {"calibration_id": calibration_id})
    assert committed["accepted"]
    assert committed["snapshot"]["controller_state"] == "calibration_valid"
    assert send("control.enable")["accepted"]
    return calibration_id


def trajectory(active_calibration_id, **overrides):
    return {
        "model_hash": "model-v1",
        "calibration_id": active_calibration_id,
        "times_s": [0.0, 1.0, 2.0],
        "positions_rad": [[0.0] * 4, [0.4, 0.0, 0.0, 0.0], [0.0] * 4],
        "velocities_rad_s": [[0.0] * 4] * 3,
        **overrides,
    }


def test_uncalibrated_feedback_never_presents_joint_position(backend, send):
    assert backend.snapshot()["controller_state"] == "disconnected"
    assert all(joint["q_joint"] is None for joint in backend.snapshot()["joints"])
    assert send("gravity.set")["error"]["code"] == "not_connected"
    send("connection.connect")
    send("lease.acquire")
    assert send("control.enable")["error"]["code"] == "calibration_required"
    assert all(joint["q_joint"] is None for joint in backend.snapshot()["joints"])


def test_capture_candidate_is_not_a_committed_calibration(backend, send):
    send("connection.connect")
    send("lease.acquire")
    captured = send("zero.capture")
    candidate_id = captured["snapshot"]["calibration_candidate"]["calibration_id"]
    assert captured["snapshot"]["calibration_id"] is None
    assert captured["snapshot"]["calibration_state"] == "captured"
    assert all(joint["q_joint"] is None for joint in captured["snapshot"]["joints"])
    assert send("zero.commit", {"calibration_id": "wrong"})["error"]["code"] == "calibration_mismatch"
    assert send("zero.commit", {"calibration_id": candidate_id})["accepted"]
    assert backend.snapshot()["controller_state"] == "calibration_valid"
    assert send("gravity.set")["error"]["code"] == "invalid_state"
    assert send("control.enable")["accepted"]


def test_calibration_pose_outside_soft_limits_can_commit_but_cannot_enable(send):
    send("connection.connect")
    send("lease.acquire")
    captured = send("zero.capture", {"reference_joint_rad": [0.0, -1.2, 0.0, 0.0]})
    assert captured["accepted"]
    candidate = captured["result"]["calibration_candidate"]
    assert send("zero.commit", {"calibration_id": candidate["calibration_id"]})["accepted"]
    assert send("control.enable")["error"]["code"] == "joint_limits"
    assert send("stop")["snapshot"]["controller_state"] == "calibration_valid"


def test_replay_is_immutable_and_cannot_change_payload_or_owner(backend, send):
    first = send("connection.connect", request_id="stable")
    send("lease.acquire")
    replay = send("connection.connect", request_id="stable")
    assert replay == first
    replay["snapshot"]["joints"][0]["q_rotor"] = 99
    assert send("connection.connect", request_id="stable") == first
    assert send("estop", request_id="stable")["error"]["code"] == "request_id_conflict"
    assert send("connection.connect", request_id="stable", client_id="other")["error"]["code"] == "request_id_conflict"
    assert backend.snapshot()["controller_state"] == "read_only"


def test_only_lease_owner_can_command_and_an_observer_can_stop(send):
    ready(send)
    assert send("lease.acquire", client_id="observer")["error"]["code"] == "lease_held"
    assert send("gravity.set", client_id="observer")["error"]["code"] == "lease_required"
    assert send("gravity.set", {"scale": 0.2})["accepted"]
    assert send("stop", client_id="observer")["snapshot"]["controller_state"] == "ready"


def test_lease_timeout_faults_active_control_and_reset_needs_enable(backend, send):
    ready(send)
    send("gravity.set", {"scale": 0.2})
    snapshot = backend.advance(5.01)
    assert snapshot["controller_state"] == "fault"
    assert snapshot["gravity_scale"] == 0
    assert snapshot["lease"] is None
    assert send("fault.reset")["snapshot"]["controller_state"] == "read_only"
    assert send("control.enable")["error"]["code"] == "lease_required"
    send("lease.acquire")
    assert send("control.enable")["accepted"]


def test_heartbeat_renews_and_release_transfers_control(backend, send):
    send("connection.connect")
    send("lease.acquire")
    backend.advance(4.0)
    assert send("lease.heartbeat")["accepted"]
    assert backend.advance(2.0)["lease"]["client_id"] == "operator"
    assert send("lease.release")["accepted"]
    assert send("lease.acquire", client_id="next")["accepted"]


def test_plan_follows_complete_timeline_without_teleporting(backend, send):
    calibration_id = ready(send)
    result = send("plan.validate", trajectory(calibration_id))
    assert result["accepted"]
    plan_id = result["result"]["plan"]["plan_id"]
    assert send("plan.execute", {"plan_id": plan_id})["accepted"]
    assert backend.snapshot()["joints"][0]["q_joint"] == 0.0
    assert backend.advance(0.5)["joints"][0]["q_joint"] == pytest.approx(0.2)
    assert backend.advance(0.5)["joints"][0]["q_joint"] == pytest.approx(0.4)
    assert backend.advance(0.5)["joints"][0]["q_joint"] == pytest.approx(0.2)
    final = backend.advance(0.5)
    assert final["joints"][0]["q_joint"] == 0.0
    assert final["controller_state"] == "ready"
    assert final["active_plan_id"] is None


def test_plan_identity_changes_with_contents_and_cannot_be_mutated(backend, send):
    calibration_id = ready(send)
    payload = trajectory(calibration_id)
    first = send("plan.validate", payload)["result"]["plan"]
    payload["positions_rad"][1][0] = 0.2
    second = send("plan.validate", payload)["result"]["plan"]
    assert first["plan_id"] != second["plan_id"]
    first["positions_rad"][1][0] = -0.8
    send("plan.execute", {"plan_id": first["plan_id"]})
    assert backend.advance(1)["joints"][0]["q_joint"] == pytest.approx(0.4)


@pytest.mark.parametrize(
    ("overrides", "code"),
    [
        ({"model_hash": "old-model"}, "identity_mismatch"),
        ({"calibration_id": "old-calibration"}, "identity_mismatch"),
        ({"times_s": [0, 0, 1]}, "invalid_plan"),
        ({"times_s": [1, 2, 3]}, "invalid_plan"),
        ({"times_s": [0, 0.1, 0.2]}, "velocity_limit"),
        ({"positions_rad": [[0.1] * 4] * 3}, "start_mismatch"),
        ({"positions_rad": [[0.0] * 4, [1.2] * 4, [0.0] * 4]}, "joint_limits"),
        ({"velocities_rad_s": [[9.0] * 4] * 3}, "velocity_limit"),
        ({"velocities_rad_s": [[float("nan")] * 4] * 3}, "invalid_payload"),
    ],
)
def test_plan_rejects_invalid_identity_shape_time_start_limits(send, overrides, code):
    calibration_id = ready(send)
    assert send("plan.validate", trajectory(calibration_id, **overrides))["error"]["code"] == code


def test_backend_collision_check_cannot_be_overridden_by_payload():
    backend = FakeArmBackend(model_hash="m", collision_validator=lambda path: False)
    index = count()
    send = lambda name, payload=None: backend.command(
        name, payload, request_id=str(next(index)), client_id="a"
    )
    calibration_id = ready(send)
    result = send("plan.validate", trajectory(calibration_id, model_hash="m", collision_checked=True))
    assert result["error"]["code"] == "collision"


def test_missing_collision_validator_fails_closed():
    backend = FakeArmBackend(model_hash="m")
    index = count()
    send = lambda name, payload=None: backend.command(
        name, payload, request_id=str(next(index)), client_id="a"
    )
    calibration_id = ready(send)
    result = send("plan.validate", trajectory(calibration_id, model_hash="m"))
    assert result["error"]["code"] == "collision_checker_required"


def test_execution_rechecks_start_and_collision(send, backend):
    calibration_id = ready(send)
    first = send("plan.validate", trajectory(calibration_id))["result"]["plan"]
    second_payload = trajectory(calibration_id)
    second_payload["positions_rad"][-1][0] = 0.4
    second = send("plan.validate", second_payload)["result"]["plan"]
    send("plan.execute", {"plan_id": second["plan_id"]})
    backend.advance(2.0)
    assert send("plan.execute", {"plan_id": first["plan_id"]})["error"]["code"] == "start_mismatch"


def test_estop_latches_and_cannot_be_cleared_by_disconnect(send):
    ready(send)
    assert send("estop", client_id="observer")["snapshot"]["controller_state"] == "estop"
    assert send("gravity.set")["accepted"] is False
    assert send("fault.reset")["snapshot"]["controller_state"] == "read_only"


def test_hardware_mode_never_presents_a_live_motor_adapter():
    backend = FakeArmBackend(model_hash="m", hardware=True)
    initial = copy.deepcopy(backend.snapshot())
    result = backend.command("connection.connect", request_id="hardware", client_id="operator")
    assert result["error"]["code"] == "hardware_adapter_unavailable"
    assert result["snapshot"]["controller_state"] == initial["controller_state"] == "disconnected"
    assert result["snapshot"]["hardware_io"] is False
