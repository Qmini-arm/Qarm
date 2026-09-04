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
from .dynamics import ArmDynamics, DynamicsSample, MotorDynamicsSimulator
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


def _simulation_table(frame: CommandFrame, sample: DynamicsSample) -> str:
    lines = [
        "| ID | 目标角 ° | 仿真角 ° | 跟踪误差 ° | 电机关节力矩 Nm | 重力负载 Nm |",
        "|---:|---:|---:|---:|---:|---:|",
    ]
    for index, motor in enumerate(frame.motors):
        target_deg = np.degrees(motor.joint_position_rad)
        actual_deg = np.degrees(sample.positions_rad[index])
        lines.append(
            f"| {motor.motor_id} | {target_deg:+.2f} | {actual_deg:+.2f} | "
            f"{target_deg - actual_deg:+.2f} | {sample.control_torque_nm[index]:+.3f} | "
            f"{sample.gravity_load_nm[index]:+.3f} |"
        )
    return "\n".join(lines)


def launch_visualization(
    model: ArmModel,
    collision: CollisionChecker,
    planner: MotionPlanner,
    mapper: M8010CommandMapper,
    dynamics: ArmDynamics,
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

    simulator = MotorDynamicsSimulator(
        dynamics,
        q0,
        gear_ratio=mapper.gear_ratio,
        directions=np.asarray([cal.direction for cal in mapper.calibrations]),
    )
    root_from_base = model.root_to_base
    base_from_root = np.linalg.inv(root_from_base)

    def pose_in_root(pose_in_base: npt.ArrayLike) -> np.ndarray:
        return root_from_base @ np.asarray(pose_in_base, dtype=np.float64).reshape(4, 4)

    def point_in_root(point_in_base: npt.ArrayLike) -> np.ndarray:
        point = np.asarray(point_in_base, dtype=np.float64).reshape(3)
        return root_from_base[:3, :3] @ point + root_from_base[:3, 3]

    def point_in_base(point_in_root_frame: npt.ArrayLike) -> np.ndarray:
        point = np.asarray(point_in_root_frame, dtype=np.float64).reshape(3)
        return base_from_root[:3, :3] @ point + base_from_root[:3, 3]

    server = viser.ViserServer(host=host, port=port)
    rendered = ViserUrdf(server, Path(model.urdf_path))
    rendered_names = tuple(rendered.get_actuated_joint_names())
    if set(rendered_names) != set(model.joint_names):
        raise ValueError("Viser URDF joint set differs from the kinematic model")

    state: dict[str, Any] = {"q": q0.copy(), "plan": None, "playing": False}
    lock = threading.RLock()
    syncing = {"active": False}
    start_pose = model.fk(q0)
    start_pose_root = pose_in_root(start_pose)
    target = server.scene.add_transform_controls(
        "/target",
        position=start_pose_root[:3, 3],
        wxyz=tf.SO3.from_matrix(start_pose_root[:3, :3]).wxyz,
        scale=0.12,
        depth_test=False,
        disable_rotations=True,
    )
    reached = server.scene.add_frame(
        "/tool0",
        position=start_pose_root[:3, 3],
        wxyz=tf.SO3.from_matrix(start_pose_root[:3, :3]).wxyz,
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

    home_to_base_root = root_from_base[:3, 3] - start_pose_root[:3, 3]
    gravity_root = dynamics.config.gravity_root_m_s2
    alignment_cosine = float(
        home_to_base_root @ gravity_root
        / (np.linalg.norm(home_to_base_root) * np.linalg.norm(gravity_root))
    )
    alignment_deg = float(np.degrees(np.arccos(np.clip(alignment_cosine, -1.0, 1.0))))
    gravity_direction = gravity_root / np.linalg.norm(gravity_root)
    server.scene.add_arrows(
        "/gravity",
        points=np.asarray(
            [[root_from_base[:3, 3], root_from_base[:3, 3] + 0.15 * gravity_direction]],
            dtype=np.float32,
        ),
        colors=(235, 70, 70),
        shaft_radius=0.004,
        head_radius=0.012,
        head_length=0.025,
    )

    with server.gui.add_folder("初步动力学仿真"):
        dynamics_toggle = server.gui.add_checkbox("启用重力与关节动力学", initial_value=True)
        gravity_compensation_toggle = server.gui.add_checkbox(
            "启用名义重力补偿（仅仿真）", initial_value=True
        )
        server.gui.add_markdown(
            f"xacro `{model.root_link}` 坐标中重力："
            f"`[{gravity_root[0]:+.5f}, {gravity_root[1]:+.5f}, {gravity_root[2]:+.5f}] m/s²`  "
            f"  \n转换到 `base_link`："
            f"`[{dynamics.gravity_base_m_s2[0]:+.5f}, {dynamics.gravity_base_m_s2[1]:+.5f}, "
            f"{dynamics.gravity_base_m_s2[2]:+.5f}] m/s²`  "
            f"  \nhome pose 末端→基座与重力的夹角：`{alignment_deg:.2f}°`。  "
            "  \n重力补偿选项只影响仿真被控对象，不会写入 CSV 或发送到真机。"
        )
        initial_frame = mapper.map_sample(0.0, q0, np.zeros(model.dof), q0)
        initial_sample = simulator.advance(initial_frame, 0.0, compensate_gravity=True)
        simulation_table = server.gui.add_markdown(_simulation_table(initial_frame, initial_sample))

    with server.gui.add_folder("六轴 M8010 控制参数"):
        calibration_note = (
            "绝对转子位置已由标定计算。"
            if mapper.absolute_positions_available
            else "⚠️ 标定文件仍为占位值；绝对转子位置不会生成，仅显示相对启动点的转子偏移。"
        )
        server.gui.add_markdown(calibration_note)
        command_table = server.gui.add_markdown(_command_table(initial_frame))

    path_handle: dict[str, Any] = {"value": None}
    workspace_handle: dict[str, Any] = {"value": None, "loading": False}

    def ordered_for_renderer(q: npt.ArrayLike) -> np.ndarray:
        values = np.asarray(q, dtype=np.float64).reshape(model.dof)
        by_name = dict(zip(model.joint_names, values, strict=True))
        return np.asarray([by_name[name] for name in rendered_names])

    def render(
        q: npt.ArrayLike, frame: CommandFrame, sample: DynamicsSample | None = None
    ) -> None:
        values = np.asarray(q, dtype=np.float64).reshape(model.dof)
        rendered.update_cfg(ordered_for_renderer(values))
        pose = pose_in_root(model.fk(values))
        reached.position = pose[:3, 3]
        reached.wxyz = tf.SO3.from_matrix(pose[:3, :3]).wxyz
        command_table.content = _command_table(frame)
        if sample is not None:
            simulation_table.content = _simulation_table(frame, sample)

    def target_position() -> np.ndarray:
        return np.asarray([slider.value for slider in target_sliders], dtype=np.float64)

    def on_target_drag(_event: object = None) -> None:
        with lock:
            if syncing["active"]:
                return
            syncing["active"] = True
            try:
                target_base = point_in_base(target.position)
                for slider, value in zip(target_sliders, target_base, strict=True):
                    slider.value = float(np.clip(value, slider.min, slider.max))
            finally:
                syncing["active"] = False

    def on_target_slider(_event: object = None) -> None:
        with lock:
            if syncing["active"]:
                return
            target.position = point_in_root(target_position())

    def draw_plan(plan: MotionPlan) -> None:
        if path_handle["value"] is not None:
            path_handle["value"].remove()
        tool_path = np.asarray(
            [point_in_root(model.fk(q)[:3, 3]) for q in plan.trajectory.positions_rad]
        )
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
            last_valid_q = np.asarray(state["q"], dtype=np.float64).copy()
        use_dynamics = False
        try:
            trajectory = plan.trajectory
            start_q = trajectory.positions_rad[0]
            use_dynamics = bool(dynamics_toggle.value)
            compensate_gravity = bool(gravity_compensation_toggle.value)
            if use_dynamics:
                simulator.reset(state["q"])
            wall_start = time.monotonic()
            previous_time_s = 0.0
            maximum_tracking_error_rad = 0.0
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
                sample = None
                rendered_q = q
                if use_dynamics:
                    sample = simulator.advance(
                        frame,
                        float(time_s) - previous_time_s,
                        compensate_gravity=compensate_gravity,
                    )
                    rendered_q = sample.positions_rad
                    maximum_tracking_error_rad = max(
                        maximum_tracking_error_rad,
                        float(np.max(np.abs(np.asarray(q) - rendered_q))),
                    )
                    if not model.within_limits(rendered_q):
                        raise RuntimeError("动力学仿真姿态越过 URDF 软限位")
                    contacts = collision.check(rendered_q)
                    if contacts:
                        raise RuntimeError(
                            "动力学仿真姿态发生自碰撞：" + ", ".join(map(str, contacts))
                        )
                render(rendered_q, frame, sample)
                last_valid_q = np.asarray(rendered_q, dtype=np.float64).copy()
                previous_time_s = float(time_s)
            with lock:
                state["q"] = last_valid_q
                if use_dynamics:
                    status.value = (
                        "动力学仿真完成；最大关节跟踪误差 "
                        f"{np.degrees(maximum_tracking_error_rad):.2f}°，"
                        "当前仿真姿态成为下一次规划起点"
                    )
                else:
                    status.value = "运动学播放完成；当前姿态成为下一次规划的起点"
        except Exception as exc:
            with lock:
                state["q"] = last_valid_q
                if use_dynamics:
                    simulator.reset(last_valid_q)
                status.value = f"播放失败：{exc}；已停在最后一个有效姿态"
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
            simulator.reset(q0)
            play_button.disabled = True
            frame = mapper.map_sample(0.0, q0, np.zeros(model.dof), q0)
            sample = simulator.advance(
                frame,
                0.0,
                compensate_gravity=bool(gravity_compensation_toggle.value),
            )
            render(q0, frame, sample)
            pose = pose_in_root(model.fk(q0))
            syncing["active"] = True
            try:
                target.position = pose[:3, 3]
                for slider, value in zip(target_sliders, model.fk(q0)[:3, 3], strict=True):
                    slider.value = float(value)
            finally:
                syncing["active"] = False
            status.value = "仿真已复位"

    def load_workspace() -> None:
        try:
            cloud = sample_workspace(model, collision, count=workspace_samples, seed=0)
            points_root = (
                cloud.positions_m @ root_from_base[:3, :3].T + root_from_base[:3, 3]
            )
            handle = server.scene.add_point_cloud(
                "/collision_free_workspace",
                points=points_root.astype(np.float32),
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
    render(q0, initial_frame, initial_sample)
    logger.info("Qmini visualization: http://%s:%d", host, port)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        server.stop()
