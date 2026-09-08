"""Viser workbench for Qarm FK and position-only IK."""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import viser

from functions import (
    forward_kinematics as functions_forward_kinematics,
)
from functions import (
    inverse_kinematics as functions_inverse_kinematics,
)

from .env import DEFAULT_MODEL_PATH, JOINT_NAMES, QArmMujocoEnv
from .kinematics import PoseResult, forward_kinematics, solve_position_ik


def _mesh_arrays(model: mujoco.MjModel, mesh_id: int) -> tuple[np.ndarray, np.ndarray]:
    vertex_start = int(model.mesh_vertadr[mesh_id])
    vertex_count = int(model.mesh_vertnum[mesh_id])
    face_start = int(model.mesh_faceadr[mesh_id])
    face_count = int(model.mesh_facenum[mesh_id])
    vertices = np.asarray(
        model.mesh_vert[vertex_start : vertex_start + vertex_count], dtype=np.float32
    ).copy()
    faces = np.asarray(
        model.mesh_face[face_start : face_start + face_count], dtype=np.uint32
    ).copy()
    return vertices, faces


class QarmViserApp:
    """Interactive FK/IK browser workbench backed by MuJoCo kinematics."""

    def __init__(
        self,
        *,
        model_path: str | Path = DEFAULT_MODEL_PATH,
        host: str = "127.0.0.1",
        port: int = 8080,
    ) -> None:
        self.environment = QArmMujocoEnv(model_path)
        self.server = viser.ViserServer(host=host, port=port, label="Qarm FK / IK")
        self._lock = threading.RLock()
        self._updating_controls = False
        self._closed = False

        self._mesh_handles: list[Any] = []
        self._build_scene()
        self._build_controls()
        self._apply_fk()

    def _build_scene(self) -> None:
        scene = self.server.scene
        scene.set_up_direction("+z")
        scene.add_grid(
            "/ground",
            width=2.0,
            height=2.0,
            plane="xy",
            cell_size=0.1,
            section_size=0.5,
            cell_color=(160, 170, 180),
            section_color=(90, 100, 110),
            plane_color=(40, 45, 52),
            plane_opacity=0.15,
            position=(0.0, 0.0, 0.0),
        )
        scene.add_frame("/world", axes_length=0.12, axes_radius=0.006)
        self._tool_frame = scene.add_frame("/tool", axes_length=0.09, axes_radius=0.005)
        self._tool_marker = scene.add_icosphere(
            "/tool/marker",
            radius=0.012,
            color=(60, 220, 140),
            position=(0.0, 0.0, 0.0),
        )
        self._target_marker = scene.add_icosphere(
            "/target/marker",
            radius=0.018,
            color=(245, 150, 45),
            position=(0.0, 0.0, 0.0),
        )

        # Visual mesh geoms are already expressed in meters in the MJCF.
        model = self.environment.model
        for geom_id in range(model.ngeom):
            if model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            if model.geom_contype[geom_id] != 0:
                continue
            mesh_id = int(model.geom_dataid[geom_id])
            vertices, faces = _mesh_arrays(model, mesh_id)
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
            if name is None:
                name = f"geom_{geom_id}"
            handle = scene.add_mesh_simple(
                f"/robot/{name}",
                vertices,
                faces,
                color=(180, 188, 198),
                material="standard",
                side="double",
                flat_shading=False,
            )
            self._mesh_handles.append((geom_id, handle))

    def _build_controls(self) -> None:
        gui = self.server.gui
        with gui.add_folder("FK / 关节空间"):
            self._joint_controls = [
                gui.add_slider(
                    f"{name} (deg)",
                    min=float(np.degrees(self.environment.model.jnt_range[joint_id, 0])),
                    max=float(np.degrees(self.environment.model.jnt_range[joint_id, 1])),
                    step=0.5,
                    initial_value=0.0,
                )
                for name, joint_id in zip(JOINT_NAMES, self.environment.joint_ids)
            ]
            for control in self._joint_controls:
                control.on_update(lambda _event: self._apply_fk())
            self._fk_button = gui.add_button("应用 FK", icon="refresh")
            self._fk_button.on_click(lambda _event: self._apply_fk())
            self._reset_button = gui.add_button("复位关节")
            self._reset_button.on_click(lambda _event: self._reset())

        with gui.add_folder("IK / 末端位置"):
            self._target_controls = [
                gui.add_number(label, 0.0, min=-1.5, max=1.5, step=0.005)
                for label in ("目标 X (m)", "目标 Y (m)", "目标 Z (m)")
            ]
            self._ik_seed_dropdown = gui.add_dropdown(
                "IK 初值",
                options=("当前 FK", "零位"),
                initial_value="当前 FK",
            )
            self._target_button = gui.add_button("将当前 FK 设为目标")
            self._target_button.on_click(lambda _event: self._set_target_from_fk())
            self._ik_button = gui.add_button("求解位置 IK", icon="target")
            self._ik_button.on_click(lambda _event: self._solve_ik())

        with gui.add_folder("functions.py / 平面参考"):
            self._functions_q1 = gui.add_number("q1 (rad)", 0.0, min=-3.2, max=3.2, step=0.01)
            self._functions_q2 = gui.add_number("q2 (rad)", 0.0, min=-3.2, max=3.2, step=0.01)
            self._functions_r = gui.add_number("目标 r (m)", 0.6, min=-0.7, max=0.7, step=0.005)
            self._functions_z = gui.add_number("目标 z (m)", 0.0, min=-0.7, max=0.7, step=0.005)
            self._functions_fk_button = gui.add_button("运行 functions.py FK")
            self._functions_fk_button.on_click(lambda _event: self._apply_functions_fk())
            self._functions_ik_button = gui.add_button("运行 functions.py IK")
            self._functions_ik_button.on_click(lambda _event: self._apply_functions_ik())
            self._functions_result = gui.add_text(
                "平面结果", initial_value="r=0, z=0", disabled=True
            )

        with gui.add_folder("结果"):
            self._status = gui.add_text("状态", initial_value="就绪", disabled=True)
            self._position_text = gui.add_text("FK 位置 (m)", initial_value="", disabled=True)
            self._orientation_text = gui.add_text(
                "FK 四元数 (wxyz)", initial_value="", disabled=True
            )
            self._target_text = gui.add_text("IK 目标 (m)", initial_value="", disabled=True)
            self._error_text = gui.add_text("IK 误差 (m)", initial_value="", disabled=True)

        self._note = gui.add_markdown(
            "MuJoCo IK 约束 XYZ；functions.py 参考模型只约束二维 r/z，不能代表完整四轴姿态。"
        )

    def _joint_position(self) -> np.ndarray:
        return np.radians(np.asarray([control.value for control in self._joint_controls]))

    def _target_position(self) -> np.ndarray:
        return np.asarray([control.value for control in self._target_controls], dtype=np.float64)

    @staticmethod
    def _format_vector(values: np.ndarray) -> str:
        return "[" + ", ".join(f"{float(value):+.5f}" for value in values) + "]"

    def _update_meshes(self) -> None:
        data = self.environment.data
        for geom_id, handle in self._mesh_handles:
            handle.position = data.geom_xpos[geom_id].copy()
            quat = np.zeros(4, dtype=np.float64)
            mujoco.mju_mat2Quat(quat, data.geom_xmat[geom_id])
            handle.wxyz = quat

    def _set_tool_marker(self, pose: PoseResult) -> None:
        self._tool_frame.position = pose.position_m
        self._tool_frame.wxyz = pose.orientation_wxyz
        self._tool_marker.position = pose.position_m
        self._target_marker.position = self._target_position()

    def _apply_fk(self) -> PoseResult | None:
        if self._updating_controls or self._closed:
            return None
        with self._lock:
            try:
                position = self._joint_position()
                pose = forward_kinematics(self.environment, position)
                with self.environment._lock:
                    self.environment.data.qpos[self.environment.qpos_addresses] = position
                    self.environment.data.qvel[self.environment.dof_addresses] = 0.0
                    mujoco.mj_forward(self.environment.model, self.environment.data)
                    self._update_meshes()
                self._set_tool_marker(pose)
                self._position_text.value = self._format_vector(pose.position_m)
                self._orientation_text.value = self._format_vector(pose.orientation_wxyz)
                self._status.value = "FK 已更新"
                return pose
            except ValueError as error:
                self._status.value = f"FK 错误: {error}"
                return None

    def _apply_functions_fk(self) -> None:
        r, z = functions_forward_kinematics(
            float(self._functions_q1.value), float(self._functions_q2.value)
        )
        self._functions_r.value = float(r)
        self._functions_z.value = float(z)
        self._functions_result.value = f"r={r:+.5f} m, z={z:+.5f} m"

    def _apply_functions_ik(self) -> None:
        q1, q2 = functions_inverse_kinematics(
            float(self._functions_r.value), float(self._functions_z.value)
        )
        if q1 is None or q2 is None:
            self._functions_result.value = "目标超出 functions.py 的二维工作空间"
            return
        self._functions_q1.value = float(q1)
        self._functions_q2.value = float(q2)
        self._functions_result.value = f"q1={q1:+.5f} rad, q2={q2:+.5f} rad"

    def _set_target_from_fk(self) -> None:
        pose = self._apply_fk()
        if pose is None:
            return
        self._updating_controls = True
        try:
            for control, value in zip(self._target_controls, pose.position_m):
                control.value = float(value)
        finally:
            self._updating_controls = False
        self._target_text.value = self._format_vector(pose.position_m)
        self._status.value = "已将 FK 位置复制到 IK 目标"

    def _reset(self) -> None:
        self._updating_controls = True
        try:
            for control in self._joint_controls:
                control.value = 0.0
        finally:
            self._updating_controls = False
        self._apply_fk()

    def _solve_ik(self) -> None:
        with self._lock:
            try:
                target = self._target_position()
                seed = self._joint_position()
                if self._ik_seed_dropdown.value == "零位":
                    seed = np.zeros(4, dtype=np.float64)
                result = solve_position_ik(
                    self.environment,
                    target,
                    seed_rad=seed,
                    position_tolerance_m=1e-4,
                )
                self._updating_controls = True
                try:
                    for control, value in zip(self._joint_controls, np.degrees(result.position_rad)):
                        control.value = float(value)
                finally:
                    self._updating_controls = False
                self._apply_fk()
                self._target_text.value = self._format_vector(target)
                self._error_text.value = self._format_vector(result.error_m)
                self._status.value = (
                    f"IK {'收敛' if result.converged else '未完全收敛'}: "
                    f"误差 {result.error_norm_m:.6f} m / {result.iterations} 次"
                )
            except ValueError as error:
                self._status.value = f"IK 错误: {error}"

    def run(self) -> None:
        print(f"Qarm Viser FK/IK: http://{self.server.get_host()}:{self.server.get_port()}")
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.close()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.environment.close()
        self.server.stop()


def run_viser(
    *, model_path: str | Path = DEFAULT_MODEL_PATH, host: str = "127.0.0.1", port: int = 8080
) -> None:
    QarmViserApp(model_path=model_path, host=host, port=port).run()
