from __future__ import annotations

import numpy as np
from qmini_arm_motion import ArmModel, CollisionChecker


def test_urdf_chain_and_zero_pose(model: ArmModel) -> None:
    assert model.base_link == "base_link"
    assert model.tip_link == "tool0"
    assert model.joint_names == tuple(f"joint_{index}" for index in range(1, 7))
    assert np.allclose(np.degrees(model.lower), -145.0, atol=1e-6)
    assert np.allclose(np.degrees(model.upper), 145.0, atol=1e-6)
    assert np.allclose(
        model.fk(np.zeros(6))[:3, 3],
        [0.01599543, -0.60800545, 0.07125505],
        atol=1e-8,
    )


def test_analytic_position_jacobian_matches_finite_difference(model: ArmModel) -> None:
    q = np.array([0.2, -0.3, 0.4, -0.25, 0.15, -0.1])
    numeric = np.zeros((3, model.dof))
    step = 1e-7
    for index in range(model.dof):
        delta = np.zeros(model.dof)
        delta[index] = step
        numeric[:, index] = (model.fk(q + delta)[:3, 3] - model.fk(q - delta)[:3, 3]) / (2.0 * step)
    assert np.allclose(model.jacobian(q)[:3], numeric, atol=1e-7)


def test_collision_checker_accepts_zero_and_rejects_known_overlap(
    collision: CollisionChecker,
) -> None:
    assert collision.is_free(np.zeros(6))
    colliding = np.array(
        [1.54375852, 1.55862840, 0.07756963, -1.08415664, -2.25775960, -0.59032314]
    )
    pairs = collision.check(colliding)
    assert pairs
    assert any({pair.link_a, pair.link_b} == {"link_4", "link_6"} for pair in pairs)


def test_edge_check_includes_the_full_segment(collision: CollisionChecker) -> None:
    colliding = np.array(
        [1.54375852, 1.55862840, 0.07756963, -1.08415664, -2.25775960, -0.59032314]
    )
    assert not collision.segment_is_free(np.zeros(6), colliding)
