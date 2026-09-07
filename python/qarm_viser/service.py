"""Presentation-independent planning and command dispatch.

GUI callbacks only enqueue work. Slow IK runs in its own worker; a single
supervisor advances the offline backend and renews the connected owner's lease.
"""

from __future__ import annotations

import queue
import threading
import time
import uuid
from concurrent.futures import Future, ThreadPoolExecutor
from typing import Any

import numpy as np

from qmini_arm_motion import ArmModel, CollisionChecker, MotionPlanner, PlannerConfig


class BackendRejected(RuntimeError):
    pass


class ControlService:
    def __init__(self, backend: Any, model: ArmModel) -> None:
        self.backend = backend
        self.model = model
        self.collision = CollisionChecker(model)
        self.planner = MotionPlanner(
            model, self.collision,
            config=PlannerConfig(velocity_limit_rad_s=0.25,
                                 acceleration_limit_rad_s2=0.5, control_period_s=0.01),
        )
        self._queue: queue.Queue = queue.Queue(maxsize=64)
        self._urgent: queue.Queue = queue.Queue(maxsize=16)
        self._workers = ThreadPoolExecutor(max_workers=1, thread_name_prefix="qarm-planning")
        self._stop = threading.Event()
        self._lock = threading.RLock()
        self._thread: threading.Thread | None = None
        self._clients: set[str] = set()
        self._heartbeat_at = 0.0
        self._revision = 0
        self._planning = False
        self.plan: dict | None = None
        self.notice = "离线控制器就绪"

    def snapshot(self) -> dict:
        value = self.backend.snapshot()
        return value.to_dict() if hasattr(value, "to_dict") else value

    def _command(self, name: str, payload: dict | None, client_id: str) -> dict:
        response = self.backend.command(
            name, payload or {}, request_id=uuid.uuid4().hex, client_id=client_id,
        )
        if not response["accepted"]:
            error = response["error"]
            raise BackendRejected(f"{error['code']}: {error['message']}")
        return response["result"]

    def client_connected(self, client_id: str) -> None:
        with self._lock:
            self._clients.add(client_id)

    def client_disconnected(self, client_id: str) -> None:
        with self._lock:
            self._clients.discard(client_id)
            # Browser refresh yields a new client. It must acquire a new lease.
            self._revision += 1

    def submit(self, name: str, payload: dict | None, client_id: str) -> Future:
        future: Future = Future()
        target = self._urgent if name in {"stop", "estop"} else self._queue
        with self._lock:
            if self._stop.is_set():
                future.set_exception(BackendRejected("控制服务已关闭"))
                return future
            try:
                target.put_nowait((future, name, payload or {}, client_id))
            except queue.Full:
                future.set_exception(BackendRejected("命令队列已满"))
        return future

    def dispatch(self, name: str, payload: dict, client_id: str) -> dict:
        """Synchronously dispatch on the supervisor thread (also used by tests)."""
        with self._lock:
            if name == "_plan.commit":
                if payload["revision"] != self._revision:
                    raise BackendRejected("规划期间状态或目标已变化，请重新规划")
                result = self._command("plan.validate", payload["trajectory"], client_id)
                self.plan = {**payload["trajectory"], **result.get("plan", result)}
                self.notice = f"计划已校验：{self.plan['plan_id']}"
                return result
            if name == "plan.execute":
                if self.plan is None:
                    raise BackendRejected("请先生成并验证计划")
                payload = {key: self.plan[key] for key in
                           ("plan_id", "model_hash", "calibration_id")}
            if name == "zero.commit":
                candidate = self.snapshot().get("calibration_candidate")
                if not candidate:
                    raise BackendRejected("没有可提交的标零候选")
                payload = {**payload, "calibration_id": candidate["calibration_id"]}
            result = self._command(name, payload, client_id)
            if name not in {"lease.heartbeat", "status"}:
                self._revision += 1
            if name in {"zero.capture", "zero.commit", "disconnect", "connection.disconnect",
                        "stop", "estop", "fault.reset", "reset"}:
                self.plan = None
            self.notice = f"已处理：{name}"
            return result

    def invalidate_plan(self) -> None:
        with self._lock:
            self._revision += 1
            self.plan = None

    def plan_async(self, target: Any, client_id: str, *, joint_space: bool = False) -> Future:
        with self._lock:
            if self._planning:
                raise BackendRejected("正在规划，请等待或停止")
            snapshot = self.snapshot()
            if self._stop.is_set() or snapshot["controller_state"] != "ready":
                raise BackendRejected("控制器必须处于 ready 状态才能规划")
            q_joint = [joint["q_joint"] for joint in snapshot["joints"]]
            if any(value is None for value in q_joint):
                raise BackendRejected("标零完成后才能使用关节反馈规划")
            if (snapshot.get("lease") or {}).get("client_id") != client_id:
                raise BackendRejected("请先取得控制权")
            start = np.asarray(q_joint, dtype=float)
            goal = np.asarray(target, dtype=float).copy()
            expected = self.model.dof if joint_space else 3
            if goal.shape != (expected,) or not np.isfinite(goal).all():
                raise ValueError(f"目标需要 {expected} 个有限数值")
            self._planning = True
            self.invalidate_plan()
            revision = self._revision
            self.notice = "正在计算限位与碰撞约束轨迹…"

        def work() -> dict:
            try:
                motion = (self.planner.plan_to_configuration(start, goal) if joint_space
                          else self.planner.plan(start, goal))
                trajectory = motion.trajectory
                payload = {
                    "joint_names": list(self.model.joint_names),
                    "model_hash": snapshot["model_hash"],
                    "calibration_id": snapshot["calibration_id"],
                    "times_s": trajectory.times_s.tolist(),
                    "positions_rad": trajectory.positions_rad.tolist(),
                    "velocities_rad_s": trajectory.velocities_rad_s.tolist(),
                    "collision_checked": True,
                    "path_kind": motion.path_kind,
                    "position_error_m": (0.0 if joint_space else motion.ik.position_error_m),
                }
                result = self.submit("_plan.commit", {
                    "revision": revision, "trajectory": payload,
                }, client_id)
                return result.result(timeout=30)
            finally:
                with self._lock:
                    self._planning = False

        return self._workers.submit(work)

    def pump(self, dt: float) -> None:
        self.backend.advance(dt)
        now = time.monotonic()
        with self._lock:
            owner = (self.snapshot().get("lease") or {}).get("client_id")
            if owner in self._clients and now - self._heartbeat_at >= 0.5:
                try:
                    self._command("lease.heartbeat", {}, owner)
                except BackendRejected:
                    pass
                self._heartbeat_at = now
        for pending in (self._urgent, self._queue):
            for _ in range(8):
                try:
                    future, name, payload, client_id = pending.get_nowait()
                except queue.Empty:
                    break
                if not future.set_running_or_notify_cancel():
                    continue
                try:
                    future.set_result(self.dispatch(name, payload, client_id))
                except Exception as error:
                    self.notice = str(error)
                    future.set_exception(error)

    def start(self) -> None:
        if self._thread is not None or self._stop.is_set():
            raise RuntimeError("控制服务不能重复启动")
        def run() -> None:
            previous = time.monotonic()
            while not self._stop.wait(0.01):
                current = time.monotonic()
                try:
                    self.pump(current - previous)
                except Exception as error:
                    self.notice = f"后端更新失败：{error}"
                previous = current
        self._thread = threading.Thread(target=run, name="qarm-supervisor", daemon=True)
        self._thread.start()

    def close(self) -> None:
        with self._lock:
            self._revision += 1
            self._clients.clear()
            self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
        # Reject pending jobs, including an IK completion waiting on the queue.
        for pending in (self._urgent, self._queue):
            while not pending.empty():
                future, *_ = pending.get_nowait()
                if not future.done():
                    future.set_exception(BackendRejected("控制服务已关闭"))
        self._workers.shutdown(wait=False, cancel_futures=True)
        try:
            self._command("estop", {}, "shutdown")
        except BackendRejected:
            pass
