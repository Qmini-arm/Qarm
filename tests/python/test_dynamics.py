from __future__ import annotations

import numpy as np
from qmini_arm_motion import (
    ArmDynamics,
    ArmModel,
    M8010CommandMapper,
    MotorDynamicsSimulator,
)


def test_root_gravity_is_transformed_to_base_link(
    model: ArmModel, dynamics: ArmDynamics
) -> None:
    expected = model.root_to_base[:3, :3].T @ np.array([0.0, 0.0, -9.80665])
    assert np.allclose(dynamics.gravity_base_m_s2, expected)

    home_in_root = model.root_to_base @ model.fk(np.zeros(model.dof))
    tool_to_base = model.root_to_base[:3, 3] - home_in_root[:3, 3]
    gravity = dynamics.config.gravity_root_m_s2
    angle_deg = np.degrees(
        np.arccos(
            np.clip(
                tool_to_base @ gravity / (np.linalg.norm(tool_to_base) * np.linalg.norm(gravity)),
                -1.0,
                1.0,
            )
        )
    )
    assert np.isclose(angle_deg, 0.660280889, atol=1e-5)


def test_mass_matrix_and_gravity_match_urdf_energy(
    model: ArmModel, dynamics: ArmDynamics
) -> None:
    q = np.array([0.2, -0.1, 0.3, 0.05])
    matrix = dynamics.mass_matrix(q)
    assert np.allclose(matrix, matrix.T, atol=1e-12)
    assert np.min(np.linalg.eigvalsh(matrix)) > 0.0

    step = 1e-6
    energy_gradient = np.empty(model.dof)
    for index in range(model.dof):
        offset = np.zeros(model.dof)
        offset[index] = step
        energy_gradient[index] = (
            dynamics.potential_energy(q + offset) - dynamics.potential_energy(q - offset)
        ) / (2.0 * step)
    assert np.allclose(dynamics.gravity_load(q), -energy_gradient, atol=1e-7)


def test_motor_pd_simulation_responds_to_gravity(
    model: ArmModel, mapper: M8010CommandMapper, dynamics: ArmDynamics
) -> None:
    q0 = np.zeros(model.dof)
    simulator = MotorDynamicsSimulator(
        dynamics,
        q0,
        gear_ratio=mapper.gear_ratio,
        directions=np.asarray([cal.direction for cal in mapper.calibrations]),
    )
    frame = mapper.map_sample(0.0, q0, np.zeros(model.dof), q0)
    sample = simulator.advance(frame, 0.25)

    assert np.max(np.abs(sample.positions_rad - q0)) > 1e-4
    assert np.all(np.isfinite(sample.accelerations_rad_s2))
    assert np.all(np.abs(sample.control_torque_nm) <= model.effort + 1e-12)
    assert np.all(sample.positions_rad >= model.hard_lower)
    assert np.all(sample.positions_rad <= model.hard_upper)

    simulator.reset(q0)
    compensated = simulator.advance(frame, 0.25, compensate_gravity=True)
    assert np.allclose(compensated.positions_rad, q0, atol=1e-10)
    assert np.allclose(
        compensated.control_torque_nm,
        -compensated.gravity_load_nm,
        atol=1e-10,
    )
