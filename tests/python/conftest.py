from __future__ import annotations

from pathlib import Path

import pytest
from qmini_arm_motion import ArmModel, CollisionChecker, M8010CommandMapper

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="session")
def model() -> ArmModel:
    return ArmModel(ROOT / "urdf" / "qmini_arm.urdf")


@pytest.fixture(scope="session")
def collision(model: ArmModel) -> CollisionChecker:
    return CollisionChecker(model)


@pytest.fixture(scope="session")
def mapper(model: ArmModel) -> M8010CommandMapper:
    return M8010CommandMapper.from_yaml(model, ROOT / "config" / "m8010_arm.yaml")
