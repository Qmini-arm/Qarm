"""Identify rejected tabletop samples without opening or commanding real hardware."""

from __future__ import annotations

from dataclasses import replace

import pytest
from test_hardware_calibration import TABLE_REFERENCE
from test_hardware_calibration import hardware_factory as hardware_factory


def capture(send):
    return send(
        "zero.capture",
        {
            "reference_joint_rad": list(TABLE_REFERENCE),
            "directions": [1, 1, 1, 1],
            "sample_count": 200,
            "confirm_direction": True,
            "external_support_confirmed": True,
            "reference_pose_confirmed": True,
        },
    )


def change_frame(monkeypatch, bus, change):
    """Inject a fault by sample number, after the factory's connect handshake."""
    original = bus.brake_all
    calls = 0

    def brake_all():
        nonlocal calls
        calls += 1
        return tuple(change(calls, list(original())))

    monkeypatch.setattr(bus, "brake_all", brake_all)
    return original


def assert_rejected(response, backend, *, reason, sample_index, motor_id):
    assert not response["accepted"]
    assert response["error"]["code"] == "capture_unstable"
    snapshot = response["snapshot"]
    diagnostic = snapshot["capture_diagnostic"]
    assert diagnostic["reason"] == reason
    assert diagnostic["sample_index"] == sample_index
    assert diagnostic["motor_id"] == motor_id
    assert diagnostic["message"] == response["error"]["message"]
    assert f"sample {sample_index}:" in diagnostic["message"]
    if motor_id is not None:
        assert f"motor_id={motor_id}" in diagnostic["message"]
    assert snapshot["calibration_candidate"] is None
    assert snapshot["calibration_candidate_id"] is None
    assert snapshot["calibration"] is None
    assert snapshot["calibration_id"] is None
    assert snapshot["controller_state"] == "read_only"
    assert all(joint["q_joint"] is None for joint in snapshot["joints"])
    assert backend.snapshot()["capture_diagnostic"] == diagnostic
    return diagnostic["message"]


@pytest.mark.parametrize(
    ("field", "value", "reason", "details"),
    [
        ("error", 8, "motor_error", ("merror=8", "expected 0")),
        ("mode", 1, "mode", ("mode=1", "expected BRAKE=0")),
        ("dq_rad_s", 0.0201, "velocity", ("0.020100 rad/s", "0.020000 rad/s")),
        ("dq_rad_s", -0.0201, "velocity", ("0.020100 rad/s", "0.020000 rad/s")),
        ("temperature_c", 60.1, "temperature", ("60.1 C", "60 C")),
    ],
)
def test_bad_sample_identifies_motor_value_and_threshold(
    hardware_factory, monkeypatch, field, value, reason, details
):
    backend, bus, send = hardware_factory()

    def inject(sample_index, frame):
        if sample_index == 7:
            frame[2] = replace(frame[2], **{field: value})
        return frame

    change_frame(monkeypatch, bus, inject)
    message = assert_rejected(capture(send), backend, reason=reason, sample_index=7, motor_id=2)
    for detail in details:
        assert detail in message
    assert all(command.mode == "BRAKE" for command in bus.commands)


@pytest.mark.parametrize("field", ["q_rad", "dq_rad_s", "tau_nm", "temperature_c"])
@pytest.mark.parametrize("value", [float("nan"), float("inf"), float("-inf")])
def test_nonfinite_sample_cannot_create_a_calibration(hardware_factory, monkeypatch, field, value):
    backend, bus, send = hardware_factory()

    def inject(sample_index, frame):
        if sample_index == 11:
            frame[1] = replace(frame[1], **{field: value})
        return frame

    change_frame(monkeypatch, bus, inject)
    assert_rejected(capture(send), backend, reason="non_finite", sample_index=11, motor_id=1)


@pytest.mark.parametrize("mismatch", ["missing", "reordered", "duplicate", "unexpected"])
def test_motor_ids_must_match_the_entire_configured_frame(hardware_factory, monkeypatch, mismatch):
    backend, bus, send = hardware_factory()

    def inject(sample_index, frame):
        if sample_index == 3:
            if mismatch == "missing":
                frame.pop()
            elif mismatch == "reordered":
                frame[0], frame[1] = frame[1], frame[0]
            elif mismatch == "duplicate":
                frame[1] = replace(frame[1], motor_id=0)
            else:
                frame[3] = replace(frame[3], motor_id=14)
        return frame

    change_frame(monkeypatch, bus, inject)
    message = assert_rejected(
        capture(send), backend, reason="motor_ids", sample_index=3, motor_id=None
    )
    assert "returned motor IDs" in message
    assert "expected (0, 1, 2, 3)" in message


def test_position_span_rejects_motion_even_when_reported_velocity_is_zero(
    hardware_factory, monkeypatch
):
    backend, bus, send = hardware_factory()
    bus.positions[3] = 0.0

    def inject(sample_index, frame):
        if sample_index >= 100:
            frame[3] = replace(frame[3], q_rad=0.0201)
        return frame

    change_frame(monkeypatch, bus, inject)
    message = assert_rejected(
        capture(send), backend, reason="position_span", sample_index=200, motor_id=3
    )
    assert "0.020100 rad" in message
    assert "0.020000 rad" in message


@pytest.mark.parametrize("velocity", [-0.02, 0.02])
def test_exact_velocity_temperature_and_position_span_limits_remain_accepted(
    hardware_factory, monkeypatch, velocity
):
    backend, bus, send = hardware_factory()
    bus.positions[2] = 0.0

    def at_limits(sample_index, frame):
        frame[2] = replace(
            frame[2],
            q_rad=0.02 if sample_index >= 100 else 0.0,
            dq_rad_s=velocity,
            temperature_c=60.0,
        )
        return frame

    change_frame(monkeypatch, bus, at_limits)
    response = capture(send)
    assert response["accepted"], response["error"]
    candidate = response["snapshot"]["calibration_candidate"]
    assert candidate["metadata"]["rotor_span_rad"][2] == 0.02
    assert backend.snapshot()["capture_diagnostic"] is None
    assert backend.snapshot()["calibration"] is None
    assert backend.snapshot()["controller_state"] == "zero_capture"


def test_successful_recapture_clears_previous_diagnostic(hardware_factory, monkeypatch):
    backend, bus, send = hardware_factory()
    bus.velocities[0] = 0.03
    assert_rejected(capture(send), backend, reason="velocity", sample_index=1, motor_id=0)
    bus.velocities[0] = 0.0
    response = capture(send)
    assert response["accepted"], response["error"]
    assert response["snapshot"]["capture_diagnostic"] is None
    assert backend.snapshot()["capture_diagnostic"] is None
    assert response["snapshot"]["calibration_candidate"] is not None


def test_snapshot_diagnostic_is_not_a_mutable_backend_reference(hardware_factory):
    backend, bus, send = hardware_factory()
    bus.velocities[0] = 0.03
    response = capture(send)
    response["snapshot"]["capture_diagnostic"]["message"] = "mutated client response"
    snapshot = backend.snapshot()
    assert "mutated" not in snapshot["capture_diagnostic"]["message"]
    snapshot["capture_diagnostic"]["reason"] = "mutated client snapshot"
    assert backend.snapshot()["capture_diagnostic"]["reason"] == "velocity"


def test_capture_uses_transport_brake_mode_instead_of_assuming_zero(hardware_factory, monkeypatch):
    backend, bus, send = hardware_factory()
    bus.brake_mode = 3

    def mode_from_sdk(sample_index, frame):
        return [replace(item, mode=3) for item in frame]

    change_frame(monkeypatch, bus, mode_from_sdk)
    response = capture(send)
    assert response["accepted"], response["error"]
    assert backend.snapshot()["capture_diagnostic"] is None
