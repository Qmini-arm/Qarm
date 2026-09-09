"""Qmini 四轴 Viser 可视化、位置 IK 和反馈回放。

这个文件迁移了另一个机械臂的 Viser 使用方式，但不复用它的六轴模型、
舵机标定或 CDSArm 协议。模型来自当前仓库的 ``description/qmini_arm.urdf``；
实机写入只调用当前仓库的 :class:`motor_driver.ArmController`。

安全边界：

* 未传 ``--enable-hardware`` 时不会导入或打开串口；
* 传入 ``--enable-hardware`` 后仍需在浏览器中再次勾选启用；
* 浏览器启用时先读取当前位置、同步滑条，并以同一姿态建立保持，不会跳到仿真初始姿态；
* 实机目标通过 ``ArmController.moveJ`` 发送，因此沿用仓库内的限位、轨迹和
  重力补偿逻辑。

安装 Viser 依赖后可直接运行：

``python3 viser_app.py --sim --mode viewer```
"""

from __future__ import annotations

import argparse
import logging
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from functools import partial
from pathlib import Path
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)

ROOT = Path(__file__).resolve().parent
DEFAULT_URDF = ROOT / "description" / "qmini_arm.urdf"
JOINT_NAMES = ("joint_1", "joint_2", "joint_3", "joint_4")
TIP_LINK = "tool0"

_AXES_LENGTH = 0.06
_AXES_RADIUS = 0.003
_WORKSPACE_SAMPLES = 1800
_WORKSPACE_MARGIN = 0.04
_CLOUD_SAMPLES = 5000
_CLOUD_POINT_SIZE = 0.002


@dataclass(frozen=True)
class PositionIKResult:
    """Result of the four-axis position-only IK solve."""

    q: np.ndarray
    position_error: float
    iterations: int
    converged: bool


class QminiKinematics:
    """URDF-backed FK and position-only IK for the current four-axis arm.

    ``yourdfpy`` is used only for the model and FK.  The interactive target has
    three coordinates while the arm has four joints, so the solver is a damped
    least-squares position solve seeded by the current browser pose.  Joint
    limits are read from the URDF and are also used by the hardware controller.
    """

    def __init__(self, urdf_path: str | Path = DEFAULT_URDF) -> None:
        try:
            import yourdfpy
        except ImportError as exc:  # pragma: no cover - depends on environment
            raise RuntimeError(
                "缺少 yourdfpy，请安装 Viser 依赖：pip install viser yourdfpy"
            ) from exc

        self.urdf_path = Path(urdf_path).expanduser().resolve()
        if not self.urdf_path.is_file():
            raise FileNotFoundError(f"找不到 URDF: {self.urdf_path}")

        # FK does not need to load meshes.  The ViserUrdf instance below loads
        # the visual meshes separately with the same resolved URDF directory.
        self.urdf = yourdfpy.URDF.load(
            self.urdf_path,
            build_scene_graph=True,
            load_meshes=False,
            filename_handler=partial(yourdfpy.filename_handler_magic, dir=self.urdf_path.parent),
        )
        self.joint_names = tuple(self.urdf.actuated_joint_names)
        if self.joint_names != JOINT_NAMES:
            raise ValueError(
                f"当前 URDF 必须是四轴 {JOINT_NAMES}，实际得到 {self.joint_names}"
            )
        if TIP_LINK not in self.urdf.link_map:
            raise ValueError(f"URDF 缺少末端 link {TIP_LINK!r}")

        joints = self.urdf.actuated_joints
        self.lower = np.asarray(
            [float(joint.limit.lower) for joint in joints], dtype=np.float64
        )
        self.upper = np.asarray(
            [float(joint.limit.upper) for joint in joints], dtype=np.float64
        )
        if np.any(~np.isfinite(self.lower)) or np.any(~np.isfinite(self.upper)):
            raise ValueError("URDF 关节限位必须是有限数")
        if np.any(self.lower >= self.upper):
            raise ValueError("URDF 关节限位必须是非空区间")

        self._lock = threading.RLock()
        self._base_link = str(self.urdf.base_link)

    @property
    def dof(self) -> int:
        return len(self.joint_names)

    @property
    def mid_range(self) -> np.ndarray:
        return 0.5 * (self.lower + self.upper)

    def clamp(self, q: np.ndarray | list[float] | tuple[float, ...]) -> np.ndarray:
        values = np.asarray(q, dtype=np.float64).reshape(self.dof)
        if not np.all(np.isfinite(values)):
            raise ValueError("关节角必须是有限数")
        return np.clip(values, self.lower, self.upper)

    def validate(self, q: np.ndarray | list[float] | tuple[float, ...]) -> np.ndarray:
        values = np.asarray(q, dtype=np.float64).reshape(self.dof)
        if not np.all(np.isfinite(values)):
            raise ValueError("关节角必须是有限数")
        if np.any(values < self.lower) or np.any(values > self.upper):
            raise ValueError("关节角超出当前 URDF 限位")
        return values

    def fk(self, q: np.ndarray | list[float] | tuple[float, ...]) -> np.ndarray:
        """Return the ``world -> tool0`` pose for a four-joint configuration."""
        values = self.validate(q)
        with self._lock:
            self.urdf.update_cfg(values)
            return np.asarray(
                self.urdf.get_transform(TIP_LINK, self._base_link), dtype=np.float64
            ).copy()

    def _position_jacobian(self, q: np.ndarray, eps: float = 1e-6) -> np.ndarray:
        jacobian = np.empty((3, self.dof), dtype=np.float64)
        for index in range(self.dof):
            delta = np.zeros(self.dof, dtype=np.float64)
            delta[index] = eps
            # A DLS iterate can sit exactly on a URDF limit.  Perturbing past
            # that limit would make ``validate`` reject the finite-difference
            # probe, so use the closest in-range one-sided approximation.
            plus = self.fk(self.clamp(q + delta))[:3, 3]
            minus = self.fk(self.clamp(q - delta))[:3, 3]
            jacobian[:, index] = (plus - minus) / (2.0 * eps)
        return jacobian

    def solve_position(
        self,
        target: np.ndarray | list[float] | tuple[float, ...],
        *,
        seed: np.ndarray | None = None,
        max_iterations: int = 80,
        tolerance: float = 1e-4,
    ) -> PositionIKResult:
        """Solve a position target while respecting all four URDF limits."""
        goal = np.asarray(target, dtype=np.float64).reshape(3)
        if not np.all(np.isfinite(goal)):
            raise ValueError("目标位置必须是有限数")
        if max_iterations <= 0 or tolerance <= 0.0:
            raise ValueError("IK迭代次数和容差必须大于0")

        q = self.mid_range.copy() if seed is None else self.clamp(seed)
        current = self.fk(q)[:3, 3]
        best_error = float(np.linalg.norm(goal - current))
        best_q = q.copy()
        damping = 2e-3

        for iteration in range(1, max_iterations + 1):
            error = goal - current
            error_norm = float(np.linalg.norm(error))
            if error_norm <= tolerance:
                return PositionIKResult(q.copy(), error_norm, iteration - 1, True)

            jacobian = self._position_jacobian(q)
            system = jacobian @ jacobian.T + (damping**2) * np.eye(3)
            try:
                step = jacobian.T @ np.linalg.solve(system, error)
            except np.linalg.LinAlgError:
                damping = min(damping * 4.0, 1.0)
                continue

            step_norm = float(np.linalg.norm(step))
            if step_norm > 0.25:
                step *= 0.25 / step_norm
            candidate = self.clamp(q + step)
            candidate_position = self.fk(candidate)[:3, 3]
            candidate_error = float(np.linalg.norm(goal - candidate_position))

            if candidate_error < best_error:
                q, current = candidate, candidate_position
                best_q, best_error = q.copy(), candidate_error
                damping = max(damping * 0.7, 1e-5)
            else:
                damping = min(damping * 2.0, 1.0)

        return PositionIKResult(best_q, best_error, max_iterations, best_error <= tolerance)

    def sample_workspace(
        self, count: int = _WORKSPACE_SAMPLES, seed: int = 0
    ) -> np.ndarray:
        if count <= 0:
            raise ValueError("workspace采样数必须大于0")
        rng = np.random.default_rng(seed)
        values = rng.uniform(self.lower, self.upper, size=(count, self.dof))
        return np.asarray([self.fk(q)[:3, 3] for q in values], dtype=np.float32)


class QarmHardwareDriver:
    """Guarded adapter from the Viser callbacks to ``ArmController.moveJ``."""

    def __init__(self, arm: Any, *, duration: float = 0.5) -> None:
        self.arm = arm
        self._duration = 0.5
        self.duration = duration
        self._enabled = False
        self._armed = False
        self._last_target: np.ndarray | None = None
        self._lock = threading.RLock()

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def duration(self) -> float:
        return self._duration

    @duration.setter
    def duration(self, value: float) -> None:
        value = float(value)
        if not 0.05 <= value <= 10.0:
            raise ValueError("MoveJ时长必须在0.05..10秒")
        self._duration = value

    @property
    def port_name(self) -> str:
        serial_port = getattr(getattr(self.arm, "ser", None), "serial", None)
        return str(getattr(serial_port, "port", "串口"))

    def read_current(self) -> np.ndarray:
        with self._lock:
            if self._enabled:
                raise RuntimeError("实机驱动启用时不能单独读取当前位置")
            values = self.arm.get_joint_positions()
            current = np.asarray(values, dtype=np.float64).reshape(4)
            if not np.all(np.isfinite(current)):
                raise ValueError("反馈关节角包含非有限数")
            self._last_target = current.copy()
            return current

    def takeover(self) -> np.ndarray:
        """Read the current pose and start a same-pose gravity-compensated hold."""
        with self._lock:
            self._enabled = False
            current = self.read_current()
            # The target is exactly the just-read pose, so enabling the browser
            # control cannot create a midpoint jump.  moveJ keeps gravity
            # compensation inside the repository's high-level controller.
            self.arm.moveJ(current.tolist(), duration=max(self._duration, 0.2))
            self._armed = True
            return current

    def enable(self) -> None:
        with self._lock:
            if not self._armed:
                raise RuntimeError("启用实机驱动前必须先接管当前位置")
            self._enabled = True

    def disable(self) -> None:
        with self._lock:
            self._enabled = False
            self._armed = False
            self._last_target = None
            # ArmController.disable() is the repository's explicit motor-off
            # path.  It is called only after the browser or CLI has authorized
            # access to the hardware controller.
            self.arm.disable()

    def command(self, q: np.ndarray) -> bool:
        with self._lock:
            if not self._enabled:
                return False
            target = np.asarray(q, dtype=np.float64).reshape(4)
            if not np.all(np.isfinite(target)):
                raise ValueError("不能向实机发送非有限关节角")
            if self._last_target is not None and np.allclose(
                target, self._last_target, atol=1e-8, rtol=0.0
            ):
                return False
            # moveJ performs the URDF-limit validation and internal gravity
            # compensation in the current Qarm repository.
            self.arm.moveJ(target.tolist(), duration=self._duration)
            self._last_target = target.copy()
            return True


class _HardwareControls:
    """Shared browser controls for the optional, explicitly enabled hardware path."""

    def __init__(
        self,
        server: Any,
        driver: QarmHardwareDriver | None,
        lock: threading.RLock,
        on_sync: Callable[[np.ndarray], None],
    ) -> None:
        self.driver = driver
        self._lock = lock
        self._on_sync = on_sync
        self._syncing = False
        self._busy = False

        folder = server.gui.add_folder("实际机械臂")
        with folder:
            self.drive = server.gui.add_checkbox("启用实机驱动", initial_value=False)
            self.duration = server.gui.add_slider(
                "MoveJ时长 (s)",
                min=0.05,
                max=10.0,
                step=0.05,
                initial_value=float(driver.duration if driver is not None else 0.5),
            )
            self.read = server.gui.add_button("读取当前位置")
            self.status = server.gui.add_text(
                "实机状态",
                initial_value=(
                    "已连接，未接管"
                    if driver is not None
                    else "模拟模式（未打开串口）"
                ),
            )

        self.drive.on_update(self._on_drive)
        self.duration.on_update(self._on_duration)
        self.read.on_click(self._on_read)
        if driver is None:
            self.drive.disabled = True
            self.duration.disabled = True
            self.read.disabled = True

    def _set_drive_value(self, value: bool) -> None:
        self._syncing = True
        try:
            self.drive.value = value
        finally:
            self._syncing = False

    def _on_duration(self, _args: object = None) -> None:
        with self._lock:
            if self.driver is None or self._syncing:
                return
            try:
                self.driver.duration = self.duration.value
            except (TypeError, ValueError) as exc:
                self.status.value = f"时长设置失败: {exc}"

    def _on_read(self, _args: object = None) -> None:
        with self._lock:
            if self.driver is None or self._busy:
                return
            if self.driver.enabled:
                self.status.value = "请先关闭实机驱动，再读取当前位置"
                return
            self._busy = True
            self.read.disabled = True
            try:
                q = self.driver.read_current()
                self._on_sync(q)
                self.status.value = "已读取当前位置（尚未发送目标）"
            except Exception as exc:
                self.status.value = f"读取失败: {exc}"
                logger.exception("读取 Qmini 当前位置失败")
            finally:
                self.read.disabled = False
                self._busy = False

    def _on_drive(self, _args: object = None) -> None:
        with self._lock:
            if self.driver is None or self._syncing or self._busy:
                return
            if not bool(self.drive.value):
                try:
                    self.driver.disable()
                    self.status.value = "实机驱动已停用，电机已关闭"
                except Exception as exc:
                    self.status.value = f"停用失败: {exc}"
                    logger.exception("停用 Qmini 实机驱动失败")
                return

            self._busy = True
            self.drive.disabled = True
            self.duration.disabled = True
            self.read.disabled = True
            try:
                # This synchronises the UI while the command gate is still
                # disabled; takeover's same-pose MoveJ is the only write here.
                q = self.driver.takeover()
                self._on_sync(q)
                self.driver.enable()
                self.status.value = f"实机驱动已启用 ({self.driver.port_name})"
            except Exception as exc:
                try:
                    self.driver.disable()
                except Exception:
                    logger.exception("接管失败后的电机关闭失败")
                self._set_drive_value(False)
                self.status.value = f"接管失败: {exc}"
                logger.exception("接管 Qmini 实机失败")
            finally:
                self.drive.disabled = False
                self.duration.disabled = False
                self.read.disabled = False
                self._busy = False

    def report_command_error(self, exc: Exception) -> None:
        with self._lock:
            if self.driver is not None:
                try:
                    self.driver.disable()
                except Exception:
                    logger.exception("发送失败后的电机关闭失败")
            self._set_drive_value(False)
            self.status.value = f"发送失败，已停用: {exc}"
            logger.exception("发送 Qmini 实机关节目标失败")


def _load_viser_model(
    urdf_path: Path, *, host: str, port: int
) -> tuple[Any, Any]:
    try:
        import viser
        from viser.extras import ViserUrdf
    except ImportError as exc:  # pragma: no cover - depends on environment
        raise RuntimeError(
            "缺少 viser/yourdfpy，请安装：pip install viser yourdfpy"
        ) from exc

    server = viser.ViserServer(host=host, port=port)
    # The default Viser camera is several metres away; this arm is roughly
    # 0.7 m tall, so choose a useful first view while leaving orbit controls
    # untouched for the browser user.
    server.initial_camera.position = (1.15, 1.15, 0.85)
    server.initial_camera.look_at = (0.0, 0.0, 0.32)
    model = ViserUrdf(server, urdf_path, load_collision_meshes=False)
    return server, model


def _add_tip_frame(server: Any, name: str, pose: np.ndarray) -> Any:
    from viser import transforms as tf

    return server.scene.add_frame(
        name,
        wxyz=tf.SO3.from_matrix(pose[:3, :3]).wxyz,
        position=pose[:3, 3],
        axes_length=_AXES_LENGTH,
        axes_radius=_AXES_RADIUS,
    )


def _update_frame(handle: Any, pose: np.ndarray) -> None:
    from viser import transforms as tf

    handle.wxyz = tf.SO3.from_matrix(pose[:3, :3]).wxyz
    handle.position = pose[:3, 3]


def _cloud_colors(points: np.ndarray) -> np.ndarray:
    radii = np.linalg.norm(points, axis=1)
    span = float(radii.max() - radii.min())
    t = (radii - radii.min()) / max(span, 1e-9)
    colors = np.empty((len(points), 3), dtype=np.uint8)
    colors[:, 0] = (35.0 + 75.0 * t).astype(np.uint8)
    colors[:, 1] = (100.0 + 130.0 * t).astype(np.uint8)
    colors[:, 2] = (180.0 + 60.0 * t).astype(np.uint8)
    return colors


def _add_workspace_toggle(server: Any, kin: QminiKinematics) -> Any:
    checkbox = server.gui.add_checkbox("显示可达空间", initial_value=False)
    state: dict[str, Any] = {"handle": None}
    lock = threading.RLock()

    def on_toggle(_args: object = None) -> None:
        with lock:
            if state["handle"] is None:
                if not checkbox.value:
                    return
                checkbox.disabled = True
                try:
                    points = kin.sample_workspace(_CLOUD_SAMPLES)
                    state["handle"] = server.scene.add_point_cloud(
                        "/reachable_workspace",
                        points=points,
                        colors=_cloud_colors(points),
                        point_size=_CLOUD_POINT_SIZE,
                    )
                finally:
                    checkbox.disabled = False
                return
            state["handle"].visible = bool(checkbox.value)

    checkbox.on_update(on_toggle)
    return checkbox


def _position_bounds(kin: QminiKinematics) -> tuple[np.ndarray, np.ndarray]:
    points = kin.sample_workspace(_WORKSPACE_SAMPLES)
    start = kin.fk(kin.mid_range)[:3, 3].astype(np.float32)
    points = np.vstack((points, start))
    return points.min(axis=0) - _WORKSPACE_MARGIN, points.max(axis=0) + _WORKSPACE_MARGIN


def _sleep_forever() -> None:
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        return


def launch_viewer(
    kin: QminiKinematics,
    *,
    arm_controller: Any | None = None,
    move_duration: float = 0.5,
    host: str = "127.0.0.1",
    port: int = 8080,
) -> None:
    """Launch joint sliders and an optional guarded MoveJ command path."""
    driver = (
        QarmHardwareDriver(arm_controller, duration=move_duration)
        if arm_controller is not None
        else None
    )
    server, model = _load_viser_model(kin.urdf_path, host=host, port=port)
    names = tuple(model.get_actuated_joint_names())
    if names != kin.joint_names:
        raise ValueError(f"Viser关节顺序与当前模型不一致: {names}")

    q0 = kin.mid_range.copy()
    lock = threading.RLock()
    syncing = {"active": False}
    sliders: dict[str, Any] = {}
    real_controls: _HardwareControls | None = None

    tip = kin.fk(q0)
    tip_frame = _add_tip_frame(server, "/tip_pose", tip)
    tip_label = server.scene.add_label(
        "/tip_label",
        f"tool0: ({tip[0, 3]:.3f}, {tip[1, 3]:.3f}, {tip[2, 3]:.3f}) m",
        position=tip[:3, 3] + np.array([0.0, 0.0, 0.05]),
    )

    def render(q: np.ndarray) -> None:
        model.update_cfg(q)
        pose = kin.fk(q)
        _update_frame(tip_frame, pose)
        tip_label.text = (
            f"tool0: ({pose[0, 3]:.3f}, {pose[1, 3]:.3f}, {pose[2, 3]:.3f}) m"
        )
        tip_label.position = pose[:3, 3] + np.array([0.0, 0.0, 0.05])

    def on_update(_args: object = None) -> None:
        with lock:
            if syncing["active"]:
                return
            q = np.asarray([float(sliders[name].value) for name in names], dtype=np.float64)
            render(q)
            if driver is not None:
                try:
                    driver.command(q)
                except Exception as exc:
                    if real_controls is not None:
                        real_controls.report_command_error(exc)
                    else:
                        logger.exception("发送 Qmini 实机关节目标失败")

    for index, name in enumerate(names):
        slider = server.gui.add_slider(
            name,
            min=float(kin.lower[index]),
            max=float(kin.upper[index]),
            step=0.005,
            initial_value=float(q0[index]),
        )
        slider.on_update(on_update)
        sliders[name] = slider

    def sync_from_q(q: np.ndarray) -> None:
        with lock:
            values = kin.clamp(q)
            syncing["active"] = True
            try:
                for index, name in enumerate(names):
                    sliders[name].value = float(values[index])
            finally:
                syncing["active"] = False
            render(values)

    reset = server.gui.add_button("回到中位姿（仅仿真）")

    def on_reset(_args: object = None) -> None:
        with lock:
            sync_from_q(q0)
            if driver is not None and driver.enabled:
                # The reset button is deliberately not a hardware command. The
                # operator must move a slider after reviewing the new pose.
                driver._last_target = None

    reset.on_click(on_reset)
    real_controls = _HardwareControls(server, driver, lock, sync_from_q)
    _add_workspace_toggle(server, kin)
    render(q0)
    logger.info("Qmini Viser viewer: http://%s:%d", host, port)
    _sleep_forever()


def launch_ik_app(
    kin: QminiKinematics,
    *,
    arm_controller: Any | None = None,
    move_duration: float = 0.5,
    host: str = "127.0.0.1",
    port: int = 8080,
) -> None:
    """Launch a draggable tool-position target with four-axis IK."""
    from viser import transforms as tf

    driver = (
        QarmHardwareDriver(arm_controller, duration=move_duration)
        if arm_controller is not None
        else None
    )
    server, model = _load_viser_model(kin.urdf_path, host=host, port=port)
    names = tuple(model.get_actuated_joint_names())
    if names != kin.joint_names:
        raise ValueError(f"Viser关节顺序与当前模型不一致: {names}")

    q0 = kin.mid_range.copy()
    pose0 = kin.fk(q0)
    lower, upper = _position_bounds(kin)
    gizmo = server.scene.add_transform_controls(
        "/target_tf",
        position=pose0[:3, 3],
        wxyz=tf.SO3.from_matrix(pose0[:3, :3]).wxyz,
        depth_test=False,
        scale=0.15,
        disable_rotations=True,
    )
    reached_frame = _add_tip_frame(server, "/reached_pose", pose0)

    position_folder = server.gui.add_folder("目标位置 (m)")
    with position_folder:
        position_sliders = [
            server.gui.add_slider(
                axis,
                min=float(lower[index]),
                max=float(upper[index]),
                step=0.001,
                initial_value=float(pose0[index, 3]),
            )
            for index, axis in enumerate(("x", "y", "z"))
        ]
    status = server.gui.add_text("IK状态", initial_value="idle")
    reset = server.gui.add_button("回到中位姿")
    _add_workspace_toggle(server, kin)

    lock = threading.RLock()
    syncing = {"active": False}
    state: dict[str, np.ndarray] = {"q": q0.copy()}
    real_controls: _HardwareControls | None = None

    def render(q: np.ndarray) -> np.ndarray:
        values = kin.validate(q)
        state["q"] = values.copy()
        model.update_cfg(values)
        pose = kin.fk(values)
        _update_frame(reached_frame, pose)
        return pose

    def target_position() -> np.ndarray:
        return np.asarray([float(slider.value) for slider in position_sliders])

    def solve_and_render(target: np.ndarray) -> None:
        result = kin.solve_position(target, seed=state["q"])
        reached = render(result.q)
        syncing["active"] = True
        try:
            gizmo.position = reached[:3, 3]
            gizmo.wxyz = tf.SO3.from_matrix(reached[:3, :3]).wxyz
        finally:
            syncing["active"] = False

        detail = f"误差={result.position_error * 1000:.2f}mm, 迭代={result.iterations}"
        status.value = ("已收敛  " if result.converged else "未完全收敛  ") + detail
        if driver is not None and result.converged:
            try:
                driver.command(result.q)
            except Exception as exc:  # noqa: BLE001 - bus implementations vary
                if real_controls is not None:
                    real_controls.report_command_error(exc)
                status.value = f"实机发送失败: {exc}"

    def sync_from_q(q: np.ndarray) -> None:
        with lock:
            values = kin.clamp(q)
            pose = kin.fk(values)
            state["q"] = values.copy()
            syncing["active"] = True
            try:
                for slider, value in zip(position_sliders, pose[:3, 3], strict=True):
                    slider.value = float(value)
                gizmo.position = pose[:3, 3]
                gizmo.wxyz = tf.SO3.from_matrix(pose[:3, :3]).wxyz
            finally:
                syncing["active"] = False
            render(values)
            status.value = "已同步当前位置"

    def on_slider_change(_args: object = None) -> None:
        with lock:
            if not syncing["active"]:
                solve_and_render(target_position())

    def on_gizmo_change(_args: object = None) -> None:
        with lock:
            if syncing["active"]:
                return
            position = np.asarray(gizmo.position, dtype=np.float64)
            syncing["active"] = True
            try:
                for slider, value in zip(position_sliders, position, strict=True):
                    slider.value = float(np.clip(value, slider.min, slider.max))
            finally:
                syncing["active"] = False
            solve_and_render(target_position())

    def on_reset(_args: object = None) -> None:
        with lock:
            sync_from_q(q0)
            solve_and_render(pose0[:3, 3])

    for slider in position_sliders:
        slider.on_update(on_slider_change)
    gizmo.on_update(on_gizmo_change)
    reset.on_click(on_reset)
    real_controls = _HardwareControls(server, driver, lock, sync_from_q)
    solve_and_render(pose0[:3, 3])
    logger.info("Qmini Viser IK: http://%s:%d", host, port)
    _sleep_forever()


def replay(
    kin: QminiKinematics,
    arm_controller: Any,
    *,
    period: float = 0.05,
    host: str = "127.0.0.1",
    port: int = 8080,
) -> None:
    """Read current feedback and animate the four-axis digital twin."""
    if period <= 0.0:
        raise ValueError("回放周期必须大于0")
    server, model = _load_viser_model(kin.urdf_path, host=host, port=port)
    tip_frame = _add_tip_frame(server, "/feedback_tip", kin.fk(kin.mid_range))
    status = server.gui.add_text("反馈状态", initial_value="只读回放")
    _add_workspace_toggle(server, kin)

    logger.info("Qmini Viser replay: http://%s:%d", host, port)
    try:
        while True:
            values = np.asarray(arm_controller.get_joint_positions(), dtype=np.float64).reshape(4)
            if not np.all(np.isfinite(values)):
                status.value = "反馈包含非有限值"
            else:
                clipped = kin.clamp(values)
                model.update_cfg(clipped)
                _update_frame(tip_frame, kin.fk(clipped))
                status.value = "反馈正常" if np.allclose(values, clipped) else "反馈超限，已裁剪显示"
            time.sleep(period)
    except KeyboardInterrupt:
        return


def _positive_float(value: str) -> float:
    try:
        number = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("必须是数字") from exc
    if number <= 0.0:
        raise argparse.ArgumentTypeError("必须大于0")
    return number


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Qmini 四轴 Viser 可视化 / IK / 回放")
    parser.add_argument("--mode", choices=("viewer", "ik", "replay"), default="viewer")
    parser.add_argument("--urdf", default=str(DEFAULT_URDF), help="当前四轴 URDF 路径")
    parser.add_argument("--host", default="127.0.0.1", help="Viser 监听地址")
    parser.add_argument("--port", type=int, default=8080, help="Viser HTTP 端口")
    parser.add_argument(
        "--device", "--serial", dest="device", help="实机串口，例如 /dev/ttyUSB0"
    )
    hardware = parser.add_mutually_exclusive_group()
    hardware.add_argument(
        "--enable-hardware",
        action="store_true",
        help="授权当前进程打开串口；仍需在浏览器中再次启用写入",
    )
    hardware.add_argument("--sim", action="store_true", help="强制只运行仿真")
    parser.add_argument(
        "--move-duration",
        type=_positive_float,
        default=0.5,
        help="每次浏览器目标的 MoveJ 时长（秒）",
    )
    parser.add_argument(
        "--replay-period",
        type=_positive_float,
        default=0.05,
        help="replay 模式反馈周期（秒）",
    )
    parser.add_argument("-v", "--verbose", action="count", default=0)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("Viser端口必须在1..65535")
    if args.device and not args.enable_hardware:
        parser.error("指定串口前必须显式传入 --enable-hardware")
    if args.mode == "replay" and not args.enable_hardware:
        parser.error("replay 模式需要 --enable-hardware 和 --device")
    if args.enable_hardware and not args.device:
        parser.error("--enable-hardware 必须同时提供 --device/--serial")

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s: %(message)s",
    )
    kin = QminiKinematics(args.urdf)

    arm_controller: Any | None = None
    if args.enable_hardware:
        from config.config import MOTOR_OFFSETS
        from motor_driver import ArmController

        arm_controller = ArmController(
            args.device,
            urdf_path=args.urdf,
            offsets=[MOTOR_OFFSETS[index] for index in range(ArmController.MOTOR_COUNT)],
        )
        serial_port = getattr(getattr(arm_controller, "ser", None), "serial", None)
        if serial_port is None or not serial_port.is_open:
            arm_controller.close()
            raise SystemExit(f"无法打开串口 {args.device}，未启动实机界面")

    try:
        if args.mode == "viewer":
            launch_viewer(
                kin,
                arm_controller=arm_controller,
                move_duration=args.move_duration,
                host=args.host,
                port=args.port,
            )
        elif args.mode == "ik":
            launch_ik_app(
                kin,
                arm_controller=arm_controller,
                move_duration=args.move_duration,
                host=args.host,
                port=args.port,
            )
        else:
            replay(
                kin,
                arm_controller,
                period=args.replay_period,
                host=args.host,
                port=args.port,
            )
    finally:
        if arm_controller is not None:
            try:
                arm_controller.disable()
            finally:
                arm_controller.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
