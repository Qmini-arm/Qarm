from pathlib import Path

import mujoco
import numpy as np
from qarm_sim.config import JointMap, MotorParameters
from qarm_sim.model import build_scene, set_mirrored_state

ROOT = Path(__file__).resolve().parents[2]


def _scene():
    return build_scene(
        xacro_path=ROOT / "description/qmini_arm.urdf.xacro",
        mapping=JointMap.load(ROOT / "config/joint_map.json"),
        motor=MotorParameters.load(ROOT / "config/m8010.json"),
    )


def test_model_has_visuals_collisions_actuators_and_tool_site() -> None:
    scene = _scene()
    assert scene.model.nq == 6
    assert scene.model.nv == 6
    assert scene.model.nu == 6
    assert scene.model.nmesh == 5
    assert scene.model.nsite >= 1
    assert scene.model.nsensor == 20
    assert np.count_nonzero(scene.model.geom_contype) == 14
    assert np.count_nonzero(scene.model.geom_contype == 0) == 13
    assert mujoco.mj_id2name(
        scene.model, mujoco.mjtObj.mjOBJ_SITE, scene.tool_site_id
    ) == "tool0"


def test_torque_actuators_are_limited_to_official_peak() -> None:
    scene = _scene()
    assert np.all(scene.model.actuator_ctrllimited)
    assert np.allclose(scene.model.actuator_ctrlrange, [-23.7, 23.7])


def test_mirror_sets_exact_joint_state() -> None:
    scene = _scene()
    position = np.array([0.1, -0.2, 0.3, -0.4, 0.5, -0.6])
    velocity = np.arange(6, dtype=float) * 0.01
    set_mirrored_state(scene, position, velocity)
    assert np.allclose(scene.data.qpos[scene.qpos_addresses], position)
    assert np.allclose(scene.data.qvel[scene.dof_addresses], velocity)
    assert np.isfinite(scene.data.site_xpos[scene.tool_site_id]).all()
