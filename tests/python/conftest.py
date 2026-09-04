from __future__ import annotations

from pathlib import Path

import pytest
from qmini_arm_motion import (
    ArmDynamics,
    ArmModel,
    CollisionChecker,
    DynamicsConfig,
    M8010CommandMapper,
)

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="session")
def model() -> ArmModel:
    return ArmModel(ROOT / "description" / "qmini_arm.urdf")


@pytest.fixture(scope="session")
def collision(model: ArmModel) -> CollisionChecker:
    return CollisionChecker(model)


@pytest.fixture(scope="session")
def mapper(model: ArmModel) -> M8010CommandMapper:
    return M8010CommandMapper.from_yaml(model, ROOT / "config" / "m8010_arm.yaml")


@pytest.fixture(scope="session")
def dynamics(model: ArmModel) -> ArmDynamics:
    return ArmDynamics(model, DynamicsConfig.from_yaml(ROOT / "config" / "m8010_arm.yaml"))
