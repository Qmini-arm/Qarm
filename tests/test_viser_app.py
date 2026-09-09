from __future__ import annotations

from pathlib import Path

import numpy as np

from viser_app import JOINT_NAMES, QarmHardwareDriver, QminiKinematics

ROOT = Path(__file__).resolve().parents[1]


def test_current_urdf_is_four_axis_and_fk_is_finite() -> None:
    kin = QminiKinematics(ROOT / "description" / "qmini_arm.urdf")

    assert kin.joint_names == JOINT_NAMES
    pose = kin.fk(kin.mid_range)
    assert pose.shape == (4, 4)
    assert np.all(np.isfinite(pose))


def test_position_ik_reaches_a_known_pose() -> None:
    kin = QminiKinematics(ROOT / "description" / "qmini_arm.urdf")
    q_target = np.array([0.25, -0.55, 0.70, -0.30])
    target = kin.fk(q_target)[:3, 3]

    result = kin.solve_position(target, seed=kin.mid_range)

    assert result.converged
    assert result.position_error < 1e-4
    assert np.linalg.norm(kin.fk(result.q)[:3, 3] - target) < 1e-4


def test_position_ik_handles_a_joint_limit_seed() -> None:
    kin = QminiKinematics(ROOT / "description" / "qmini_arm.urdf")
    q_limit = kin.lower.copy()
    target = kin.fk(q_limit)[:3, 3]

    result = kin.solve_position(target, seed=q_limit)

    assert result.converged
    assert result.position_error == 0.0


class FakeArm:
    def __init__(self) -> None:
        self.targets: list[tuple[list[float], float]] = []
        self.disabled = 0

    def get_joint_positions(self) -> list[float]:
        return [0.0, 0.0, 0.0, 0.0]

    def moveJ(self, target: list[float], duration: float = 1.0) -> None:
        self.targets.append((target, duration))

    def disable(self) -> None:
        self.disabled += 1


class StreamingFakeArm(FakeArm):
    def __init__(self) -> None:
        super().__init__()
        self.stream_starts: list[tuple[list[float], float]] = []
        self.stream_updates: list[list[float]] = []

    def start_streaming(self, target: list[float], duration: float = 1.0) -> None:
        self.stream_starts.append((target, duration))

    def update_stream_target(self, target: list[float]) -> None:
        self.stream_updates.append(target)


def test_hardware_driver_requires_takeover_and_uses_movej() -> None:
    arm = FakeArm()
    driver = QarmHardwareDriver(arm, duration=0.4)
    q = driver.read_current()

    assert np.allclose(q, 0.0)
    assert not driver.command(q)

    driver.takeover()
    driver.enable()
    target = np.array([0.1, -0.2, 0.3, -0.1])
    assert driver.command(target)
    assert arm.targets == [([0.0, 0.0, 0.0, 0.0], 0.4), (target.tolist(), 0.4)]
    assert not driver.command(target)

    driver.disable()
    assert arm.disabled == 1
    assert not driver.command(target + 0.1)


def test_hardware_driver_streams_targets_without_restarting_movej() -> None:
    arm = StreamingFakeArm()
    driver = QarmHardwareDriver(arm, duration=0.4)

    driver.takeover()
    driver.enable()
    target = np.array([0.1, -0.2, 0.3, -0.1])
    assert driver.command(target)
    assert driver.command(target + 0.1)

    assert arm.targets == []
    assert arm.stream_starts == [([0.0, 0.0, 0.0, 0.0], 0.4)]
    assert arm.stream_updates == [target.tolist(), (target + 0.1).tolist()]


def test_hardware_driver_rejects_duration_outside_ui_range() -> None:
    arm = FakeArm()

    for duration in (0.01, 10.1):
        try:
            QarmHardwareDriver(arm, duration=duration)
        except ValueError as exc:
            assert "0.05..10" in str(exc)
        else:
            raise AssertionError("out-of-range MoveJ duration was accepted")
