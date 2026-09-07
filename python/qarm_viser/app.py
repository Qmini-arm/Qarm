"""The single browser control surface. No GUI callback touches motor hardware."""

from __future__ import annotations

import argparse
import hashlib
import json
import logging
import threading
import time
from concurrent.futures import Future
from pathlib import Path
from typing import Any

import numpy as np
import viser
from qarm_control import FakeArmBackend, HardwareArmBackend
from qmini_arm_motion import ArmModel, CollisionChecker
from qmini_arm_motion.commands import M8010CommandMapper
from qmini_arm_motion.workspace import sample_workspace
from viser.extras import ViserUrdf

from .service import BackendRejected, ControlService

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_URDF = PROJECT_ROOT / "description" / "qmini_arm.urdf"
DEFAULT_MOTOR_CONFIG = PROJECT_ROOT / "config" / "m8010_arm.yaml"
DEFAULT_CALIBRATION_POSE = PROJECT_ROOT / "config" / "calibration_pose.json"


def load_table_reference(model: ArmModel, path: Path = DEFAULT_CALIBRATION_POSE) -> np.ndarray:
    raw = json.loads(path.read_text(encoding="utf-8"))
    if tuple(raw.get("joint_names", ())) != model.joint_names:
        raise ValueError("calibration pose joint order does not match the URDF chain")
    reference = np.asarray(raw["reference_joint_rad"], dtype=float)
    if reference.shape != (model.dof,) or not np.isfinite(reference).all():
        raise ValueError("calibration pose must contain one finite angle per joint")
    if np.any(reference < model.hard_lower) or np.any(reference > model.hard_upper):
        raise ValueError("tabletop reference exceeds the URDF hard limits")
    if raw.get("validated") is not True:
        logging.warning("桌面标零位仍标记为未完成实体验证；仅允许支撑状态下采集")
    return reference


def make_fake_backend(model: ArmModel, mapper: M8010CommandMapper) -> FakeArmBackend:
    """Bind the model and independent backend collision check to the simulation."""
    identity = hashlib.sha256(model.urdf_path.read_bytes()).hexdigest()
    return FakeArmBackend(
        model_hash=identity,
        joint_names=model.joint_names,
        motor_ids=[item.motor_id for item in mapper.calibrations],
        joint_limits_rad=np.column_stack((model.lower, model.upper)),
        velocity_limits_rad_s=[mapper.joint_velocity_limit_rad_s] * model.dof,
        gear_ratio=mapper.gear_ratio,
        collision_validator=CollisionChecker(model),
        within_limits=model.within_limits,
    )


class QarmViserApp:
    def __init__(
        self,
        model: ArmModel,
        mapper: M8010CommandMapper,
        *,
        backend: Any = None,
        host: str = "127.0.0.1",
        port: int = 8080,
        start_q: Any = None,
        workspace_samples: int = 8000,
        device: str = "/dev/ttyUSB0",
        hardware: bool = False,
        sdk_path: str | None = None,
    ) -> None:
        self.model = model
        self.mapper = mapper
        self.reference_q = np.asarray(
            np.zeros(model.dof) if start_q is None else start_q,
            dtype=float,
        )
        if self.reference_q.shape != (model.dof,) or not np.isfinite(self.reference_q).all():
            raise ValueError(f"start_q must contain {model.dof} finite joint angles")
        if not model.within_limits(self.reference_q):
            raise ValueError("simulation start pose must lie inside the soft limits")
        self.table_reference_q = load_table_reference(model)
        if backend is None and hardware:
            identity = hashlib.sha256(model.urdf_path.read_bytes()).hexdigest()
            backend = HardwareArmBackend(
                device=device,
                model_hash=identity,
                joint_names=model.joint_names,
                motor_ids=[item.motor_id for item in mapper.calibrations],
                joint_limits_rad=np.column_stack((model.lower, model.upper)),
                velocity_limits_rad_s=[mapper.joint_velocity_limit_rad_s] * model.dof,
                collision_validator=CollisionChecker(model),
                within_limits=model.within_limits,
                gear_ratio=mapper.gear_ratio,
                kp_rotor=mapper.kp_rotor,
                kd_rotor=mapper.kd_rotor,
                sdk_path=sdk_path,
            )
        self.service = ControlService(backend or make_fake_backend(model, mapper), model)
        self.is_hardware = self.service.snapshot().get("backend") == "unitree_go_m8010_6"
        self._closed = threading.Event()
        self._workspace_started = False
        self._workspace_samples = workspace_samples
        self._shown_plan: str | None = None
        self._preview_started = 0.0
        self.server = viser.ViserServer(host=host, port=port, label="Qarm · Viser")
        self.server.gui.configure_theme(control_layout="collapsible", dark_mode=True)
        self.server.scene.set_up_direction("+z")
        self.server.scene.add_grid("/ground", width=1.5, height=1.5, cell_size=0.05)
        self.actual_root = self.server.scene.add_frame("/actual", show_axes=False, visible=False)
        self.ghost_root = self.server.scene.add_frame("/preview", show_axes=False)
        self.actual = ViserUrdf(self.server, model.urdf_path, root_node_name="/actual")
        self.ghost = ViserUrdf(
            self.server,
            model.urdf_path,
            root_node_name="/preview",
            mesh_color_override=(0.25, 0.7, 1.0, 0.3),
        )
        # The uncalibrated preview shows the configured tabletop reference,
        # never the URDF mathematical zero (which is an upright pose).
        self._set_configuration(self.ghost, self.table_reference_q)
        self.tool = self.server.scene.add_frame("/tool-feedback", axes_length=0.05, visible=False)
        initial_target = model.fk(self.table_reference_q)[:3, 3]
        self.gizmo = self.server.scene.add_transform_controls(
            "/target",
            position=self.to_world(initial_target),
            scale=0.12,
            disable_rotations=True,
        )
        self._build_gui(initial_target)
        self.server.on_client_connect(self._on_connect)
        self.server.on_client_disconnect(self._on_disconnect)
        self.service.start()
        self.refresh()

    def to_world(self, points: Any) -> np.ndarray:
        values = np.asarray(points)
        return values @ self.model.root_to_base[:3, :3].T + self.model.root_to_base[:3, 3]

    def to_base(self, points: Any) -> np.ndarray:
        return (np.asarray(points) - self.model.root_to_base[:3, 3]) @ self.model.root_to_base[
            :3, :3
        ]

    def _set_configuration(self, robot: ViserUrdf, q: Any) -> None:
        robot.update_cfg(np.asarray(q, dtype=float))

    @staticmethod
    def _client_id(event: Any) -> str:
        if event.client is None:
            raise BackendRejected("控制命令必须来自已连接的浏览器")
        return str(event.client.client_id)

    def _observe(self, future: Future) -> None:
        def done(result: Future) -> None:
            try:
                result.result()
            except Exception as error:
                self.service.notice = str(error)

        future.add_done_callback(done)

    def _action(self, event: Any, command: str, payload: dict | None = None) -> None:
        try:
            self._observe(self.service.submit(command, payload, self._client_id(event)))
        except Exception as error:
            self.service.notice = str(error)

    def _plan(self, event: Any, joint_space: bool = False) -> None:
        try:
            target = (
                np.radians([slider.value for slider in self.joint_targets])
                if joint_space
                else self.target.value
            )
            self._observe(
                self.service.plan_async(
                    target,
                    self._client_id(event),
                    joint_space=joint_space,
                )
            )
        except Exception as error:
            self.service.notice = str(error)

    def _button(self, label: str, command: str, payload: dict | None = None):
        button = self.server.gui.add_button(label)
        button.on_click(lambda event: self._action(event, command, payload))
        return button

    def _build_gui(self, initial_target: np.ndarray) -> None:
        gui = self.server.gui
        mode = (
            "真实电机后端 · 连接后会发送 BRAKE，须机械支撑"
            if self.is_hardware
            else "离线运动学仿真 · **不连接电机**"
        )
        gui.add_markdown(
            f"## Qarm 控制台\n{mode}\n\n"
            "标零映射 v2：桌面参考角 ↔ 采集转子角（含减速比）\n\n"
            "蓝色透明模型是参考姿态/计划预览，实体模型才是已标定反馈。"
        )
        self.status = gui.add_markdown("")
        with gui.add_folder("连接与控制权"):
            self._button(
                "连接真实电机（BRAKE）" if self.is_hardware else "连接离线控制器",
                "connection.connect",
            )
            self._button("取得控制权（当前浏览器）", "lease.acquire")
            self._button("释放控制权", "lease.release")
            self._button("断开", "connection.disconnect")
            self._button("故障 / 急停复位", "fault.reset")
        # Stop is always accessible, including to observers. Backend remains authoritative.
        self._button("停止当前动作", "stop")
        self._button("急停（锁存）", "estop")
        with gui.add_folder("标零"):
            gui.add_markdown(
                "参考姿态固定为配置中的**桌面标零位**。真机必须先外部支撑、确认方向，再采集稳定反馈；"
                "BRAKE 不提供机械抱闸。离线后端只生成候选，不写入真机文件。"
            )
            self.calibration_status = gui.add_markdown("")
            gui.add_markdown(
                "桌面参考角 (°)：`"
                + ", ".join(f"{angle:.5f}" for angle in np.degrees(self.table_reference_q))
                + "`\n\n"
                "标零是建立角度对应关系，不是把当前关节角设为 0°，也不会驱动到桌面。"
                "此姿态的 J2 超出常规软限位，标零后仍禁止直接使能。"
            )
            self.calibration_mapping = gui.add_markdown("尚未采集转子参考读数。")
            self.direction_confirmed = gui.add_checkbox("已确认参考姿态与方向", initial_value=False)
            self.capture_button = gui.add_button("采集桌面标零候选")
            self.capture_button.on_click(
                lambda event: self._action(
                    event,
                    "zero.capture",
                    {
                        "reference_joint_rad": self.table_reference_q.tolist(),
                        "directions": [item.direction for item in self.mapper.calibrations],
                        "sample_count": 200,
                        "confirm_direction": self.direction_confirmed.value,
                    },
                )
            )
            self.commit_button = gui.add_button("确认并提交候选")
            self.commit_button.on_click(
                lambda event: self._action(
                    event,
                    "zero.commit",
                    {
                        "confirm_direction": self.direction_confirmed.value,
                        "confirm_reference_pose": self.direction_confirmed.value,
                    },
                )
            )
            self._button("使能高层控制", "control.enable", {"enabled": True})
            self._button("撤销使能", "control.enable", {"enabled": False})
        with gui.add_folder("重力补偿"):
            gui.add_markdown("离线模式只验证状态切换与保持姿态；不模拟重力动力学。")
            self.gravity_scale = gui.add_slider(
                "补偿比例", min=0.0, max=1.0, step=0.05, initial_value=0.2
            )
            gui.add_button("开启补偿").on_click(
                lambda event: self._action(
                    event,
                    "gravity.set",
                    {
                        "enabled": True,
                        "scale": self.gravity_scale.value,
                    },
                )
            )
            self._button("关闭补偿", "gravity.set", {"enabled": False})
        with gui.add_folder("关节 / IK 目标"):
            gui.add_markdown("目标位置采用 base_link 坐标（米）；4 自由度只求位置 IK，不约束姿态。")
            self.target = gui.add_vector3(
                "XYZ · base_link (m)", initial_value=initial_target, step=0.001
            )
            self.target.on_update(self._target_changed)
            self.gizmo.on_update(self._gizmo_changed)
            self.joint_targets = [
                gui.add_slider(
                    name + " (°)",
                    min=float(np.degrees(low)),
                    max=float(np.degrees(high)),
                    step=0.1,
                    initial_value=float(np.degrees(q)),
                )
                for name, low, high, q in zip(
                    self.model.joint_names,
                    self.model.lower,
                    self.model.upper,
                    self.reference_q,
                    strict=True,
                )
            ]
            for slider in self.joint_targets:
                slider.on_update(lambda _event: self.service.invalidate_plan())
            gui.add_button("规划 XYZ 目标").on_click(lambda event: self._plan(event))
            gui.add_button("规划关节目标").on_click(lambda event: self._plan(event, True))
            self.execute_button = self._button("执行已验证计划", "plan.execute")
            self.preview = gui.add_checkbox("播放计划预览（不执行）", initial_value=False)
            self.plan_status = gui.add_markdown("尚无计划")
        with gui.add_folder("反馈与工作空间"):
            self.feedback = gui.add_markdown("")
            gui.add_button("采样可达工作空间").on_click(self._sample_workspace)

    def _target_changed(self, event: Any) -> None:
        if event.client is None:
            return
        self.gizmo.position = self.to_world(self.target.value)
        self.service.invalidate_plan()

    def _gizmo_changed(self, event: Any) -> None:
        if event.client is None:
            return
        self.target.value = tuple(self.to_base(self.gizmo.position))
        self.service.invalidate_plan()

    def _on_connect(self, client: viser.ClientHandle) -> None:
        self.service.client_connected(str(client.client_id))
        client.camera.position = (0.7, -0.7, 0.5)
        client.camera.look_at = (0.0, 0.0, 0.2)

    def _on_disconnect(self, client: viser.ClientHandle) -> None:
        self.service.client_disconnected(str(client.client_id))

    def _sample_workspace(self, _event: Any) -> None:
        if self._workspace_started:
            return
        self._workspace_started = True
        self.service.notice = "正在后台采样工作空间…"

        def work() -> None:
            try:
                # Separate model work from the command supervisor and planning worker.
                workspace = sample_workspace(
                    self.model, self.service.collision, count=self._workspace_samples
                )
                if not self._closed.is_set():
                    self.server.scene.add_point_cloud(
                        "/workspace",
                        points=self.to_world(workspace.positions_m),
                        colors=(70, 140, 190),
                        point_size=0.002,
                    )
                    self.service.notice = (
                        f"工作空间：{workspace.accepted_samples} 个无自碰撞样本（离散近似）"
                    )
            except Exception as error:
                self.service.notice = str(error)
                self._workspace_started = False

        threading.Thread(target=work, name="qarm-workspace", daemon=True).start()

    def refresh(self) -> None:
        snapshot = self.service.snapshot()
        state = snapshot["controller_state"]
        owner = (snapshot.get("lease") or {}).get("client_id", "无")
        self.status.content = f"状态：`{state}` · 控制浏览器：`{owner}`\n\n{self.service.notice}"
        candidate = snapshot.get("calibration_candidate")
        self.calibration_status.content = (
            f"标定：`{snapshot.get('calibration_id') or '未标定'}`\n\n"
            f"候选：`{candidate['calibration_id'] if candidate else '无'}`"
        )
        capture_error = snapshot.get("capture_diagnostic")
        if capture_error:
            self.calibration_status.content += (
                "\n\n采集被拒绝（未生成新标零候选）：\n\n" + capture_error["message"]
            )
        self.commit_button.disabled = not (candidate and self.direction_confirmed.value)
        self.capture_button.disabled = not self.direction_confirmed.value or state != "read_only"
        calibration = candidate or snapshot.get("calibration")
        if calibration:
            ratio = calibration["gear_ratio"]
            mapping_lines = [
                f"减速比 `{ratio:g}` · 参考转子读数均为 SDK 转子侧弧度\n",
                "| 关节 | 桌面参考 ° | 采集转子 rad | 当前关节 ° |",
                "|---|---:|---:|---:|",
            ]
            for index, joint in enumerate(snapshot["joints"]):
                reference = calibration["reference_joint_rad"][index]
                rotor_reference = (
                    calibration["zero_offsets_rad"][index]
                    + calibration["directions"][index] * ratio * reference
                )
                angle = joint["q_joint"]
                current = "未提交" if angle is None else f"{np.degrees(angle):.5f}"
                mapping_lines.append(
                    f"| {joint['name']} | {np.degrees(reference):.5f} | "
                    f"{rotor_reference:.5f} | {current} |"
                )
            self.calibration_mapping.content = "\n".join(mapping_lines)
        else:
            self.calibration_mapping.content = "尚未采集转子参考读数。"
        q = [joint["q_joint"] for joint in snapshot["joints"]]
        calibrated = all(value is not None for value in q) and state != "disconnected"
        self.actual_root.visible = calibrated
        self.tool.visible = calibrated
        if calibrated:
            self._set_configuration(self.actual, q)
            pose = self.model.root_to_base @ self.model.fk(q)
            self.tool.position = pose[:3, 3]
            self.tool.wxyz = viser.transforms.SO3.from_matrix(pose[:3, :3]).wxyz
        lines = ["| 关节 | 转子 rad | 关节 ° |", "|---|---:|---:|"]
        for joint in snapshot["joints"]:
            angle = "未标定" if joint["q_joint"] is None else f"{np.degrees(joint['q_joint']):.2f}"
            rotor = "—" if joint["q_rotor"] is None else f"{joint['q_rotor']:.3f}"
            lines.append(f"| {joint['name']} | {rotor} | {angle} |")
        self.feedback.content = "\n".join(lines)
        plan = self.service.plan
        self.execute_button.disabled = not (plan and state == "ready")
        if plan is None:
            self.plan_status.content = "尚无有效计划；改变目标或标定后需要重新规划。"
            if self._shown_plan is not None:
                self.server.scene.remove_by_name("/planned-path")
            self._shown_plan = None
            self.ghost_root.visible = not calibrated
            self._set_configuration(self.ghost, self.table_reference_q)
            return
        times = np.asarray(plan["times_s"])
        positions = np.asarray(plan["positions_rad"])
        if self._shown_plan != plan["plan_id"]:
            # Display-only FK subsampling bounds render work for long plans.
            samples = positions[
                np.unique(np.linspace(0, len(positions) - 1, min(160, len(positions))).astype(int))
            ]
            points = self.to_world([self.model.fk(value)[:3, 3] for value in samples])
            self.server.scene.add_line_segments(
                "/planned-path",
                points=np.stack((points[:-1], points[1:]), axis=1),
                colors=(70, 180, 255),
                line_width=3.0,
            )
            self._shown_plan = plan["plan_id"]
            self._preview_started = time.monotonic()
        elapsed = (
            (time.monotonic() - self._preview_started) % max(times[-1], 0.001)
            if self.preview.value
            else times[-1]
        )
        preview_q = [np.interp(elapsed, times, positions[:, i]) for i in range(self.model.dof)]
        self._set_configuration(self.ghost, preview_q)
        self.ghost_root.visible = True
        self.plan_status.content = (
            f"计划 `{plan['plan_id']}` · {times[-1]:.2f} s · {len(times)} 帧\n\n"
            "已检查软限位、速度与离散自碰撞。"
        )

    def run(self) -> None:
        try:
            while not self._closed.wait(0.1):
                self.refresh()
        finally:
            self.close()

    def close(self) -> None:
        if self._closed.is_set():
            return
        self._closed.set()
        self.service.close()
        self.server.stop()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Qarm unified Viser control console")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--motor-config", type=Path, default=DEFAULT_MOTOR_CONFIG)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--backend", choices=("fake", "hardware"), default="fake")
    parser.add_argument("--device", default="/dev/ttyUSB0")
    parser.add_argument("--sdk-path")
    parser.add_argument("--start-deg", type=float, nargs="+")
    parser.add_argument("--workspace-samples", type=int, default=8000)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.WARNING)
    try:
        if args.workspace_samples <= 0:
            raise ValueError("--workspace-samples must be positive")
        model = ArmModel(args.urdf)
        mapper = M8010CommandMapper.from_yaml(model, args.motor_config)
        app = QarmViserApp(
            model,
            mapper,
            host=args.host,
            port=args.port,
            start_q=None if args.start_deg is None else np.radians(args.start_deg),
            workspace_samples=args.workspace_samples,
            device=args.device,
            hardware=args.backend == "hardware",
            sdk_path=args.sdk_path,
        )
        app.run()
    except KeyboardInterrupt:
        return 0
    except (OSError, ValueError, RuntimeError) as error:
        logging.error("%s", error)
        return 1
    return 0
