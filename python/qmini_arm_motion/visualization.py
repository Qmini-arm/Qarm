"""Browser visualization for workspace, IK plans and M8010 command frames."""

from __future__ import annotations

import logging
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt

from .collision import CollisionChecker
from .commands import CommandFrame, M8010CommandMapper
from .model import ArmModel
from .planner import MotionPlan, MotionPlanner
from .workspace import sample_workspace

logger = logging.getLogger(__name__)


def _command_table(frame: CommandFrame) -> str:
    lines = [
        f"**轨迹时间：{frame.time_s:.2f} s**",
        "",
        "| ID | 关节角 ° | 关节速度 rad/s | 转子位置 rad | 启动点转子偏移 rad | "
        "转子速度 rad/s | Kp | Kd | τff Nm |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for motor in frame.motors:
        absolute = (
            "未标定" if motor.rotor_position_rad is None else f"{motor.rotor_position_rad:.3f}"
        )
        lines.append(
            f"| {motor.motor_id} | {np.degrees(motor.joint_position_rad):+.2f} | "
            f"{motor.joint_velocity_rad_s:+.3f} | {absolute} | "
            f"{motor.rotor_offset_from_start_rad:+.3f} | {motor.rotor_velocity_rad_s:+.3f} | "
            f"{motor.kp_rotor:.3f} | {motor.kd_rotor:.3f} | {motor.torque_ff_nm:+.3f} |"
        )
    return "\n".join(lines)


def _workspace_colors(points: npt.NDArray[np.float64]) -> npt.NDArray[np.uint8]:
    radii = np.linalg.norm(points, axis=1)
    scale = (radii - radii.min()) / max(float(np.ptp(radii)), 1e-12)
    return np.column_stack(
        [50.0 + 80.0 * scale, 110.0 + 120.0 * scale, 230.0 - 80.0 * scale]
    ).astype(np.uint8)


def launch_visualization(
    model: ArmModel,
    collision: CollisionChecker,
    planner: MotionPlanner,
    mapper: M8010CommandMapper,
    *,
    initial_q: npt.ArrayLike | None = None,
    workspace_samples: int = 20000,
    host: str = "0.0.0.0",
    port: int = 8080,
) -> None:
    """Launch a Viser app; it computes commands but never opens the motor port."""
    import viser
    from viser import transforms as tf
    from viser.extras import ViserUrdf

    q0 = np.zeros(model.dof) if initial_q is None else np.asarray(initial_q, dtype=np.float64)
    q0 = q0.reshape(model.dof)
    if not model.within_limits(q0) or not collision.is_free(q0):
        raise ValueError("visualization initial configuration is invalid or self-colliding")

    server = viser.ViserServer(host=host, port=port)
    rendered = ViserUrdf(server, Path(model.urdf_path))
    rendered_names = tuple(rendered.get_actuated_joint_names())
    if set(rendered_names) != set(model.joint_names):
        raise ValueError("Viser URDF joint set differs from the kinematic model")

    state: dict[str, Any] = {"q": q0.copy(), "plan": None, "playing": False}
    lock = threading.RLock()
    syncing = {"active": False}
    start_pose = model.fk(q0)
    target = server.scene.add_transform_controls(
        "/target",
        position=start_pose[:3, 3],
        wxyz=tf.SO3.from_matrix(start_pose[:3, :3]).wxyz,
        scale=0.12,
        depth_test=False,
        disable_rotations=True,
    )
    reached = server.scene.add_frame(
        "/tool0",
        position=start_pose[:3, 3],
        wxyz=tf.SO3.from_matrix(start_pose[:3, :3]).wxyz,
        axes_length=0.08,
        axes_radius=0.003,
    )

    with server.gui.add_folder("目标点 / base_link (m)"):
        target_sliders = tuple(
            server.gui.add_slider(
                axis,
                min=-0.8,
                max=0.8,
                step=0.001,
                initial_value=float(start_pose[index, 3]),
            )
            for index, axis in enumerate(("x", "y", "z"))
        )
        plan_button = server.gui.add_button("规划到目标", color="blue")
        play_button = server.gui.add_button("播放规划", disabled=True, color="green")
        reset_button = server.gui.add_button("复位仿真")
        workspace_toggle = server.gui.add_checkbox("显示无自碰撞可达空间", initial_value=False)
        status = server.gui.add_text("状态", initial_value="等待目标", disabled=True)

    with server.gui.add_folder("六轴 M8010 控制参数"):
        calibration_note = (
            "绝对转子位置已由标定计算。"
            if mapper.absolute_positions_available
            else "⚠️ 标定文件仍为占位值；绝对转子位置不会生成，仅显示相对启动点的转子偏移。"
        )
        server.gui.add_markdown(calibration_note)
        initial_frame = mapper.map_sample(0.0, q0, np.zeros(model.dof), q0)
        command_table = server.gui.add_markdown(_command_table(initial_frame))

    path_handle: dict[str, Any] = {"value": None}
    workspace_handle: dict[str, Any] = {"value": None, "loading": False}

    def ordered_for_renderer(q: npt.ArrayLike) -> np.ndarray:
        values = np.asarray(q, dtype=np.float64).reshape(model.dof)
        by_name = dict(zip(model.joint_names, values, strict=True))
        return np.asarray([by_name[name] for name in rendered_names])

    def render(q: npt.ArrayLike, frame: CommandFrame) -> None:
        values = np.asarray(q, dtype=np.float64).reshape(model.dof)
        rendered.update_cfg(ordered_for_renderer(values))
        pose = model.fk(values)
        reached.position = pose[:3, 3]
        reached.wxyz = tf.SO3.from_matrix(pose[:3, :3]).wxyz
        command_table.content = _command_table(frame)

    def target_position() -> np.ndarray:
        return np.asarray([slider.value for slider in target_sliders], dtype=np.float64)

    def on_target_drag(_event: object = None) -> None:
        with lock:
            if syncing["active"]:
                return
            syncing["active"] = True
            try:
                for slider, value in zip(target_sliders, target.position, strict=True):
                    slider.value = float(np.clip(value, slider.min, slider.max))
            finally:
                syncing["active"] = False

    def on_target_slider(_event: object = None) -> None:
        with lock:
            if syncing["active"]:
                return
            target.position = target_position()

    def draw_plan(plan: MotionPlan) -> None:
        if path_handle["value"] is not None:
            path_handle["value"].remove()
        tool_path = np.asarray([model.fk(q)[:3, 3] for q in plan.trajectory.positions_rad])
        segments = np.stack((tool_path[:-1], tool_path[1:]), axis=1)
        path_handle["value"] = server.scene.add_line_segments(
            "/planned_tool_path",
            points=segments.astype(np.float32),
            colors=(40, 220, 110),
            thickness=3.0,
            thickness_units="screen",
        )

    def on_plan(_event: object = None) -> None:
        with lock:
            if state["playing"]:
                status.value = "轨迹播放中，不能重新规划"
                return
            plan_button.disabled = True
            play_button.disabled = True
            status.value = "正在做碰撞约束 IK 与路径规划…"
            try:
                plan = planner.plan(state["q"], target_position())
                state["plan"] = plan
                draw_plan(plan)
                status.value = (
                    f"规划成功：{plan.path_kind}，{len(plan.waypoints_rad)} 个路点，"
                    f"{plan.trajectory.duration_s:.2f} s，末端误差 "
                    f"{plan.ik.position_error_m * 1000.0:.2f} mm"
                )
                play_button.disabled = False
            except Exception as exc:
                state["plan"] = None
                status.value = f"规划失败：{exc}"
                logger.exception("motion planning failed")
            finally:
                plan_button.disabled = False

    def playback() -> None:
        with lock:
            plan = state["plan"]
            if plan is None or state["playing"]:
                return
            state["playing"] = True
            plan_button.disabled = True
            play_button.disabled = True
        try:
            trajectory = plan.trajectory
            start_q = trajectory.positions_rad[0]
            wall_start = time.monotonic()
            for time_s, q, qd in zip(
                trajectory.times_s,
                trajectory.positions_rad,
                trajectory.velocities_rad_s,
                strict=True,
            ):
                delay = float(time_s) - (time.monotonic() - wall_start)
                if delay > 0.0:
                    time.sleep(delay)
                frame = mapper.map_sample(float(time_s), q, qd, start_q)
                render(q, frame)
            with lock:
                state["q"] = trajectory.positions_rad[-1].copy()
                status.value = "轨迹播放完成；当前姿态成为下一次规划的起点"
        except Exception as exc:
            status.value = f"播放失败：{exc}"
            logger.exception("trajectory playback failed")
        finally:
            with lock:
                state["playing"] = False
                plan_button.disabled = False
                play_button.disabled = state["plan"] is None

    def on_play(_event: object = None) -> None:
        threading.Thread(target=playback, name="qmini-playback", daemon=True).start()

    def on_reset(_event: object = None) -> None:
        with lock:
            if state["playing"]:
                status.value = "请等待轨迹播放完成后再复位"
                return
            state["q"] = q0.copy()
            state["plan"] = None
            play_button.disabled = True
            frame = mapper.map_sample(0.0, q0, np.zeros(model.dof), q0)
            render(q0, frame)
            pose = model.fk(q0)
            syncing["active"] = True
            try:
                target.position = pose[:3, 3]
                for slider, value in zip(target_sliders, pose[:3, 3], strict=True):
                    slider.value = float(value)
            finally:
                syncing["active"] = False
            status.value = "仿真已复位"

    def load_workspace() -> None:
        try:
            cloud = sample_workspace(model, collision, count=workspace_samples, seed=0)
            handle = server.scene.add_point_cloud(
                "/collision_free_workspace",
                points=cloud.positions_m.astype(np.float32),
                colors=_workspace_colors(cloud.positions_m),
                point_size=0.006,
                point_shape="circle",
                visible=bool(workspace_toggle.value),
            )
            with lock:
                workspace_handle["value"] = handle
                status.value = (
                    f"可达空间：{cloud.accepted_samples}/{cloud.requested_samples} "
                    "个无自碰撞 FK 样本"
                )
        except Exception as exc:
            status.value = f"可达空间采样失败：{exc}"
            logger.exception("workspace sampling failed")
        finally:
            workspace_handle["loading"] = False
            workspace_toggle.disabled = False

    def on_workspace(_event: object = None) -> None:
        with lock:
            if workspace_handle["value"] is not None:
                workspace_handle["value"].visible = bool(workspace_toggle.value)
                return
            if not workspace_toggle.value or workspace_handle["loading"]:
                return
            workspace_handle["loading"] = True
            workspace_toggle.disabled = True
            status.value = "后台采样无自碰撞可达空间…"
            threading.Thread(target=load_workspace, name="qmini-workspace", daemon=True).start()

    for slider in target_sliders:
        slider.on_update(on_target_slider)
    target.on_update(on_target_drag)
    plan_button.on_click(on_plan)
    play_button.on_click(on_play)
    reset_button.on_click(on_reset)
    workspace_toggle.on_update(on_workspace)
    render(q0, initial_frame)
    logger.info("Qmini visualization: http://%s:%d", host, port)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        server.stop()
