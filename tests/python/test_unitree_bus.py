"""Unitree Python transport tests; none opens a real motor serial port."""

from __future__ import annotations

from enum import IntEnum
from types import SimpleNamespace

import pytest
from qarm_control.unitree_bus import (
    MotorCommand,
    UnitreeBusConfig,
    UnitreeBusError,
    UnitreeM8010Bus,
)


class MotorType(IntEnum):
    GO_M8010_6 = 0


class MotorMode(IntEnum):
    BRAKE = 0
    FOC = 1
    CALIBRATE = 2


class Cmd:
    def __init__(self):
        self.motorType = None
        self.id = 0
        self.mode = 0
        self.q = self.dq = self.kp = self.kd = self.tau = 0.0


class Data:
    def __init__(self):
        self.motorType = None
        self.motor_id = 0
        self.mode = 0
        self.temp = 30
        self.merror = 0
        self.tau = self.q = self.dq = 0.0
        self.correct = False


class FakeSerial:
    instances: list[FakeSerial] = []

    def __init__(self, device: str):
        self.device = device
        self.commands = []
        self.fail = False
        self.__class__.instances.append(self)

    def sendRecv(self, command, data):
        self.commands.append(command)
        if self.fail:
            return False
        data.motor_id = command.id
        data.mode = command.mode
        data.q = command.q + 0.125
        data.dq = command.dq
        data.tau = command.tau
        data.correct = True
        return True

    def close(self):
        return None


SDK = SimpleNamespace(
    SerialPort=FakeSerial,
    MotorCmd=Cmd,
    MotorData=Data,
    MotorType=MotorType,
    MotorMode=MotorMode,
    queryMotorMode=lambda _kind, mode: int(mode),
)


def make_bus(tmp_path, *, path="/dev/null"):
    return UnitreeM8010Bus(
        UnitreeBusConfig(device=path, lock_path=str(tmp_path / "bus.lock")),
        sdk_module=SDK,
    )


def test_invalid_device_is_rejected_before_serial_factory(tmp_path):
    called = False

    def factory(_device):
        nonlocal called
        called = True
        raise AssertionError("SerialPort must not be constructed")

    config = UnitreeBusConfig(
        device=str(tmp_path / "regular-file"), lock_path=str(tmp_path / "bus.lock")
    )
    (tmp_path / "regular-file").write_text("not a tty")
    with pytest.raises(UnitreeBusError, match="character device"):
        UnitreeM8010Bus(config, sdk_module=SDK, serial_factory=factory)
    assert not called


def test_brake_is_default_and_feedback_is_normalized(tmp_path):
    bus = make_bus(tmp_path)
    try:
        feedback = bus.exchange(MotorCommand(motor_id=2))
        assert feedback.motor_id == 2
        assert feedback.q_rad == pytest.approx(0.125)
        assert FakeSerial.instances[-1].commands[-1].mode == int(MotorMode.BRAKE)
    finally:
        bus.close()


def test_foc_fields_are_forwarded_without_unit_conversion(tmp_path):
    bus = make_bus(tmp_path)
    try:
        bus.exchange(
            MotorCommand(
                motor_id=1, q_rad=2.0, dq_rad_s=-3.0, kp=0.2, kd=0.01, tau_nm=0.4, mode="FOC"
            )
        )
        command = FakeSerial.instances[-1].commands[-1]
        assert (command.id, command.q, command.dq, command.kp, command.kd, command.tau) == (
            1,
            2.0,
            -3.0,
            0.2,
            0.01,
            0.4,
        )
        assert command.mode == int(MotorMode.FOC)
    finally:
        bus.close()


def test_timeout_and_reply_id_are_fail_closed(tmp_path):
    bus = make_bus(tmp_path)
    try:
        serial = FakeSerial.instances[-1]
        serial.fail = True
        with pytest.raises(UnitreeBusError, match="timeout"):
            bus.exchange(MotorCommand(motor_id=0))
    finally:
        bus.close()


def test_motor_command_rejects_invalid_values():
    with pytest.raises(ValueError):
        MotorCommand(motor_id=15)
    with pytest.raises(ValueError):
        MotorCommand(motor_id=0, q_rad=float("nan"))
    with pytest.raises(ValueError):
        MotorCommand(motor_id=0, mode="UNKNOWN")
