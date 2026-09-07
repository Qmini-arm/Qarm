"""Hardware-domain calibration regressions with an entirely in-memory motor bus.

These tests never construct a vendor SDK object or open a serial device.  The
feedback deliberately stays independent of commanded targets, as on a real bus.
"""

from __future__ import annotations

from itertools import count

import pytest
from qarm_control.hardware_backend import HardwareArmBackend
from qarm_control.unitree_bus import MotorCommand, MotorFeedback

TABLE_REFERENCE = (0.0, 1.7480178111, 0.1548064707, 0.0)
CAPTURED_ROTOR = (3.25, -12.4, 7.1, -2.8)


class MemoryMotorBus:
    """Return prescribed feedback, never mirror a command into measured q."""

    def __init__(self):
        self.positions = list(CAPTURED_ROTOR)
        self.velocities = [0.0] * 4
        self.commands: list[MotorCommand] = []
        self.closed = False

    def exchange(self, command):
        assert not self.closed
        self.commands.append(command)
        motor_id = command.motor_id
        return MotorFeedback(
            motor_id=motor_id,
            q_rad=self.positions[motor_id],
            dq_rad_s=self.velocities[motor_id],
            tau_nm=0.0,
            temperature_c=25.0,
            error=0,
            mode=0,
        )

    def brake_all(self):
        return tuple(self.exchange(MotorCommand(motor_id=index)) for index in range(4))

    def close(self):
        self.closed = True


@pytest.fixture
def hardware_factory(monkeypatch):
    created = []

    def forbid_serial(*args, **kwargs):
        pytest.fail("hardware calibration tests must not construct a serial transport")

    monkeypatch.setattr("qarm_control.hardware_backend.UnitreeM8010Bus", forbid_serial)
    monkeypatch.setattr(HardwareArmBackend, "_start_io", lambda self: None)

    def create(*, gear_ratio=6.33):
        backend = HardwareArmBackend(
            device="/not-a-serial-device/qarm-calibration-test",
            motor_ids=(0, 1, 2, 3),
            model_hash="table-calibration-test-model",
            gear_ratio=gear_ratio,
            joint_limits_rad=((-3.0, 3.0), (-1.57, 1.57), (-2.0, 2.0), (-3.0, 3.0)),
            collision_validator=lambda path: True,
        )
        bus = MemoryMotorBus()
        backend._bus = bus
        sequence = count()

        def send(name, payload=None):
            return backend.command(
                name, payload, request_id=f"hardware-test-{next(sequence)}", client_id="operator"
            )

        assert send("connection.connect")["accepted"]
        assert send("lease.acquire")["accepted"]
        created.append(backend)
        return backend, bus, send

    yield create
    for backend in created:
        backend.close()


def capture_and_commit(send, directions):
    captured = send(
        "zero.capture",
        {
            "reference_joint_rad": list(TABLE_REFERENCE),
            "directions": list(directions),
            "sample_count": 200,
            "confirm_direction": True,
            "external_support_confirmed": True,
            "reference_pose_confirmed": True,
        },
    )
    assert captured["accepted"], captured.get("error")
    assert all(joint["q_joint"] is None for joint in captured["snapshot"]["joints"])
    candidate = captured["result"]["calibration_candidate"]
    committed = send(
        "zero.commit",
        {"calibration_id": candidate["calibration_id"], "confirm_direction": True},
    )
    assert committed["accepted"], committed.get("error")
    return candidate


def measured_q(backend):
    return [joint["q_joint"] for joint in backend.snapshot()["joints"]]


@pytest.mark.parametrize("directions", [(1, 1, 1, 1), (-1, 1, -1, 1)])
@pytest.mark.parametrize("gear_ratio", [6.33, 7.5])
def test_table_capture_corresponds_to_the_same_measured_rotor_pose(
    hardware_factory, directions, gear_ratio
):
    backend, bus, send = hardware_factory(gear_ratio=gear_ratio)
    candidate = capture_and_commit(send, directions)
    expected_offsets = [
        rotor - direction * gear_ratio * reference
        for rotor, direction, reference in zip(
            CAPTURED_ROTOR, directions, TABLE_REFERENCE, strict=True
        )
    ]
    assert candidate["zero_offsets_rad"] == pytest.approx(expected_offsets, abs=1e-10)
    assert candidate["gear_ratio"] == gear_ratio
    assert candidate["metadata"]["rotor_at_reference_rad"] == pytest.approx(CAPTURED_ROTOR)
    assert measured_q(backend) == pytest.approx(TABLE_REFERENCE, abs=1e-10)

    # First physical feedback after commit must not snap from the table pose to
    # q_reference / 6.33, the previous rotor-vs-joint-unit regression.
    backend._io_cycle()
    assert measured_q(backend) == pytest.approx(TABLE_REFERENCE, abs=1e-10)
    assert backend.snapshot()["controller_state"] == "calibration_valid"
    assert all(command.mode == "BRAKE" for command in bus.commands)


def test_measured_rotor_displacement_and_velocity_use_direction_and_gear_ratio(hardware_factory):
    backend, bus, send = hardware_factory()
    directions = (-1, 1, -1, 1)
    capture_and_commit(send, directions)
    displacement = (0.1, -0.5, 0.2, -0.3)
    joint_velocity = (0.02, -0.03, 0.04, -0.05)
    bus.positions = [
        rotor + direction * 6.33 * delta
        for rotor, direction, delta in zip(CAPTURED_ROTOR, directions, displacement, strict=True)
    ]
    bus.velocities = [
        direction * 6.33 * velocity
        for direction, velocity in zip(directions, joint_velocity, strict=True)
    ]
    backend._io_cycle()
    assert measured_q(backend) == pytest.approx(
        [reference + delta for reference, delta in zip(TABLE_REFERENCE, displacement, strict=True)],
        abs=1e-10,
    )
    snapshot = backend.snapshot()
    assert [joint["dq_joint"] for joint in snapshot["joints"]] == pytest.approx(joint_velocity)

    # In ready mode the inverse transform must return the measured rotor q for
    # a hold target, including negative motor directions.
    assert send("control.enable")["accepted"]
    backend._io_cycle()
    commands = bus.commands[-4:]
    assert all(command.mode == "FOC" for command in commands)
    assert [command.q_rad for command in commands] == pytest.approx(bus.positions)


def test_table_capture_does_not_enable_motion_outside_soft_limits(hardware_factory):
    backend, bus, send = hardware_factory()
    capture_and_commit(send, (1, 1, 1, 1))
    backend._io_cycle()
    response = send("control.enable")
    assert not response["accepted"]
    assert response["error"]["code"] == "joint_limits"
    backend._io_cycle()
    assert backend.snapshot()["controller_state"] == "calibration_valid"
    assert measured_q(backend) == pytest.approx(TABLE_REFERENCE)
    assert all(command.mode == "BRAKE" for command in bus.commands)


def test_trajectory_target_never_replaces_measured_hardware_joint_position(hardware_factory):
    backend, bus, send = hardware_factory()
    candidate = capture_and_commit(send, (1, 1, 1, 1))
    # A supported, externally repositioned arm is now within normal limits.
    bus.positions[1] += 6.33 * (1.0 - TABLE_REFERENCE[1])
    backend._io_cycle()
    start = measured_q(backend)
    assert send("control.enable")["accepted"]
    target = [start[0] + 0.2, *start[1:]]
    validation = send(
        "plan.validate",
        {
            "model_hash": "table-calibration-test-model",
            "calibration_id": candidate["calibration_id"],
            "times_s": [0.0, 1.0],
            "positions_rad": [start, target],
            "velocities_rad_s": [[0.0] * 4, [0.0] * 4],
        },
    )
    assert validation["accepted"], validation.get("error")
    plan = validation["result"]["plan"]
    assert send(
        "plan.execute",
        {
            "plan_id": plan["plan_id"],
            "model_hash": plan["model_hash"],
            "calibration_id": plan["calibration_id"],
        },
    )["accepted"]
    backend.advance(0.5)
    assert measured_q(backend) == pytest.approx(start)
    backend._io_cycle()
    assert measured_q(backend) == pytest.approx(start)
    # The target can advance while feedback stays still; only the target is
    # converted to the command's rotor coordinates.
    assert bus.commands[-4].q_rad == pytest.approx(CAPTURED_ROTOR[0] + 6.33 * 0.1)
