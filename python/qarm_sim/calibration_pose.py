from __future__ import annotations

import math
from dataclasses import dataclass

import mujoco
import numpy as np
from numpy.typing import NDArray

from qarm_sim.model import ArmScene, build_scene


@dataclass(frozen=True)
class CalibrationPoseSolution:
    joint_position_rad: NDArray[np.float64]
    table_z_m: float
    arm_link_gap_m: float
    motor_3_gap_m: float
    motor_4_axis_vertical_error_deg: float
    motor_5_table_clearance_m: float

    @property
    def joint_position_deg(self) -> NDArray[np.float64]:
        return np.degrees(self.joint_position_rad)


def _visual_mesh_geom(scene: ArmScene, body: str, mesh: str) -> int:
    model = scene.model
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
    if body_id < 0:
        raise ValueError(f"MuJoCo model has no body {body!r}")
    for geom_id in range(model.ngeom):
        if int(model.geom_bodyid[geom_id]) != body_id:
            continue
        if model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        if int(model.geom_contype[geom_id]) != 0:
            continue
        mesh_id = int(model.geom_dataid[geom_id])
        mesh_name = mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_MESH, mesh_id
        )
        if mesh_name == mesh:
            return geom_id
    raise ValueError(f"body {body!r} has no visual mesh {mesh!r}")


def _collision_cylinder_geom(scene: ArmScene, body: str) -> int:
    model = scene.model
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
    matches = [
        geom_id
        for geom_id in range(model.ngeom)
        if int(model.geom_bodyid[geom_id]) == body_id
        and model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_CYLINDER
        and int(model.geom_contype[geom_id]) != 0
    ]
    if len(matches) != 1:
        raise ValueError(
            f"expected one collision cylinder on {body!r}, found {len(matches)}"
        )
    return matches[0]


def _mesh_world_vertices(
    scene: ArmScene, geom_id: int
) -> NDArray[np.float64]:
    model = scene.model
    data = scene.data
    mesh_id = int(model.geom_dataid[geom_id])
    start = int(model.mesh_vertadr[mesh_id])
    count = int(model.mesh_vertnum[mesh_id])
    local = model.mesh_vert[start : start + count]
    rotation = data.geom_xmat[geom_id].reshape(3, 3)
    return local @ rotation.T + data.geom_xpos[geom_id]


def _set_pose(scene: ArmScene, q: NDArray[np.float64]) -> None:
    scene.data.qpos[scene.qpos_addresses] = q
    scene.data.qvel[scene.dof_addresses] = 0.0
    mujoco.mj_forward(scene.model, scene.data)


def _bisect_root(function, lower: float, upper: float) -> float:
    f_lower = float(function(lower))
    f_upper = float(function(upper))
    if f_lower == 0.0:
        return lower
    if f_upper == 0.0:
        return upper
    if f_lower * f_upper > 0.0:
        raise ValueError(
            f"STL tangency root is not bracketed: f({lower})={f_lower}, "
            f"f({upper})={f_upper}"
        )
    for _ in range(64):
        middle = 0.5 * (lower + upper)
        f_middle = float(function(middle))
        if f_lower * f_middle <= 0.0:
            upper = middle
        else:
            lower = middle
            f_lower = f_middle
    return 0.5 * (lower + upper)


def solve_table_supported_pose(
    scene: ArmScene | None = None,
) -> CalibrationPoseSolution:
    """Solve the accepted manual calibration pose from the actual STL vertices.

    The positive branch matches the operator's setup: joint_2 moves
    counterclockwise about 100 degrees, the first arm rests on the table, and
    the downstream motor rests at the far end. Motor ID 5 is manually
    approached clockwise to the joint_6 lower hard limit.
    Normal URDF limits are intentionally not applied to this manual-only pose.
    """

    scene = build_scene() if scene is None else scene
    base_pair = _visual_mesh_geom(scene, "base_link", "base_pair")
    first_arm = _visual_mesh_geom(scene, "link_2", "arm_link")
    motor_3 = _visual_mesh_geom(scene, "link_3", "motor")
    motor_4_axis_geom = _collision_cylinder_geom(scene, "link_4")
    motor_5 = _visual_mesh_geom(scene, "link_5", "motor")
    q = np.zeros(6, dtype=np.float64)
    _set_pose(scene, q)
    table_z = float(_mesh_world_vertices(scene, base_pair)[:, 2].min())

    def first_arm_gap(joint_2: float) -> float:
        q.fill(0.0)
        q[1] = joint_2
        _set_pose(scene, q)
        return float(
            _mesh_world_vertices(scene, first_arm)[:, 2].min() - table_z
        )

    # The operator selected the counterclockwise positive branch. Unlike the
    # mirrored negative solution, it keeps the upward motor-4 solution within
    # joint_4's normal hard limit.
    q[1] = _bisect_root(first_arm_gap, math.radians(90), math.radians(120))

    def motor_3_gap(joint_3: float) -> float:
        q[2] = joint_3
        _set_pose(scene, q)
        return float(
            _mesh_world_vertices(scene, motor_3)[:, 2].min() - table_z
        )

    q[2] = _bisect_root(motor_3_gap, 0.0, math.radians(30))
    def motor_4_axis_z(joint_4: float) -> float:
        q[3] = joint_4
        _set_pose(scene, q)
        cylinder_axis = scene.data.geom_xmat[motor_4_axis_geom].reshape(
            3, 3
        )[:, 2]
        return float(cylinder_axis[2])

    # Axis Z is C + A*cos(q4) + B*sin(q4). Solve its maximum analytically;
    # this selects the upward horizontal branch and is deterministic even
    # when acos is numerically flat near the optimum.
    z_zero = motor_4_axis_z(0.0)
    z_pi = motor_4_axis_z(math.pi)
    z_half_pi = motor_4_axis_z(0.5 * math.pi)
    constant = 0.5 * (z_zero + z_pi)
    cosine = 0.5 * (z_zero - z_pi)
    sine = z_half_pi - constant
    q[3] = math.atan2(sine, cosine)
    q[4] = 0.0
    q[5] = float(scene.model.jnt_range[scene.joint_ids[5], 0])
    _set_pose(scene, q)
    arm_gap = float(
        _mesh_world_vertices(scene, first_arm)[:, 2].min() - table_z
    )
    downstream_gap = float(
        _mesh_world_vertices(scene, motor_3)[:, 2].min() - table_z
    )

    motor_axis = scene.data.geom_xmat[motor_4_axis_geom].reshape(3, 3)[:, 2]
    vertical_alignment = float(np.clip(motor_axis[2], -1.0, 1.0))
    horizontal_face_error = math.degrees(math.acos(vertical_alignment))
    motor_5_clearance = float(
        _mesh_world_vertices(scene, motor_5)[:, 2].min() - table_z
    )
    return CalibrationPoseSolution(
        joint_position_rad=q.copy(),
        table_z_m=table_z,
        arm_link_gap_m=arm_gap,
        motor_3_gap_m=downstream_gap,
        motor_4_axis_vertical_error_deg=horizontal_face_error,
        motor_5_table_clearance_m=motor_5_clearance,
    )
