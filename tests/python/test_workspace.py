from __future__ import annotations

import numpy as np
from qmini_arm_motion import ArmModel, CollisionChecker, sample_workspace


def test_workspace_contains_only_collision_free_fk_samples(
    model: ArmModel, collision: CollisionChecker
) -> None:
    workspace = sample_workspace(model, collision, count=250, seed=9)
    assert 0 < workspace.accepted_samples <= workspace.requested_samples
    assert workspace.configurations_rad.shape == (workspace.accepted_samples, model.dof)
    assert workspace.positions_m.shape == (workspace.accepted_samples, 3)
    for index in range(min(20, workspace.accepted_samples)):
        q = workspace.configurations_rad[index]
        assert collision.is_free(q)
        assert np.allclose(model.fk(q)[:3, 3], workspace.positions_m[index])
