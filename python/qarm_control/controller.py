"""Pure Python safety controller shared by Viser and the socket worker.

The class intentionally owns no serial device.  A worker may feed it
``MotorFeedback`` frames and forward the ``MotorCommand`` tuple returned by
``step`` to a bus adapter.
"""
from __future__ import annotations

import hashlib
import json
import math
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import yaml

from qmini_arm_motion import ArmDynamics, ArmModel, CollisionChecker, DynamicsConfig

from .unitree_bus import MotorCommand, MotorFeedback


@dataclass(frozen=True)
class ControllerConfig:
    urdf_path: str
    motor_config_path: str
    rotor_torque_cap_nm: float = 2.0
    max_velocity_rad_s: float = 2.0
    max_acceleration_rad_s2: float = 1.0
    control_period_s: float = 0.02
    feedback_timeout_s: float = 0.1
    lease_timeout_s: float = 5.0
    calibration_samples: int = 200
    gear_ratio: float = 6.33
    torque_slew_nm_s: float = 0.5


class _Reject(Exception):
    def __init__(self, code: str, message: str):
        self.code, self.message = code, message
        super().__init__(message)


def _finite_vector(value: Any, n: int, name: str) -> np.ndarray:
    try:
        arr = np.asarray(value, dtype=float).reshape(-1)
    except Exception as exc:
        raise _Reject("invalid_payload", f"{name} must be numeric") from exc
    if arr.size != n or not np.all(np.isfinite(arr)):
        raise _Reject("invalid_payload", f"{name} must contain {n} finite values")
    return arr


class ArmController:
    """Feedback-driven arm state machine with no hardware side effects."""

    _lease_commands = {"zero.capture", "zero.commit", "control.enable", "gravity.set",
                       "plan.validate", "plan.execute", "fault.reset"}
    _commands = {"connection.connect", "connection.disconnect", "lease.acquire",
                 "lease.heartbeat", "lease.release", "zero.capture", "zero.commit",
                 "control.enable", "gravity.set", "plan.validate", "plan.execute",
                 "stop", "estop", "fault.reset"}

    def __init__(self, config: ControllerConfig, clock=time.monotonic):
        self.config, self._clock = config, clock
        if config.gear_ratio <= 0 or config.rotor_torque_cap_nm <= 0:
            raise ValueError("invalid safety configuration")
        self.model = ArmModel(config.urdf_path)
        self.collision = CollisionChecker(self.model)
        self.dynamics = ArmDynamics(self.model, DynamicsConfig.from_yaml(config.motor_config_path))
        self.n = self.model.dof
        raw = yaml.safe_load(Path(config.motor_config_path).read_text(encoding="utf-8")) or {}
        joints = raw.get("joints", [])
        self.motor_ids = tuple(int(j.get("motor_id", i)) for i, j in enumerate(joints)) if joints else tuple(range(self.n))
        self.directions_config = tuple(int(j.get("direction", 1)) for j in joints) if joints else (1,) * self.n
        if len(self.motor_ids) != self.n or any(d not in (-1, 1) for d in self.directions_config):
            raise ValueError("motor configuration does not match URDF")
        self.joint_names = tuple(self.model.joint_names)
        self.model_hash = hashlib.sha256(Path(config.urdf_path).read_bytes() + Path(config.motor_config_path).read_bytes()).hexdigest()[:16]
        self.board_boot_id = "python-controller-" + hashlib.sha256(f"{id(self)}:{time.time_ns()}".encode()).hexdigest()[:12]
        self._lock = threading.RLock()
        self._state = "disconnected"
        self._sequence = 0
        self._feedback: dict[int, MotorFeedback] = {}
        self._feedback_time = 0.0
        self._targets = np.zeros(self.n)
        self._calibration: dict[str, Any] | None = None
        self._candidate: dict[str, Any] | None = None
        self._capture_ref: np.ndarray | None = None
        self._capture_dir: tuple[int, ...] | None = None
        self._capture_values: list[np.ndarray] = []
        self._lease_client: str | None = None
        self._lease_until = 0.0
        self._plans: dict[str, dict[str, Any]] = {}
        self._active_plan: dict[str, Any] | None = None
        self._elapsed = 0.0
        self._gravity = False
        self._gravity_scale = 0.0
        self._fault: str | None = None
        self._estop = False
        self._last_tau = np.zeros(self.n)
        self._responses: dict[str, tuple[str, dict[str, Any]]] = {}
        self._revision = 0

    def _now(self) -> float:
        return float(self._clock())

    def _fresh(self, now: float | None = None) -> bool:
        return bool(self._feedback) and self._feedback_time >= 0 and (self._now() if now is None else now) - self._feedback_time <= self.config.feedback_timeout_s

    def _healthy(self) -> bool:
        return len(self._feedback) == self.n and all(f.error == 0 and math.isfinite(f.q_rad) and math.isfinite(f.dq_rad_s) for f in self._feedback.values())

    def _joint_q(self) -> np.ndarray:
        if self._calibration is None:
            return np.full(self.n, np.nan)
        mean = np.asarray(self._calibration["rotor_mean"])
        ref = np.asarray(self._calibration["reference_joint_rad"])
        direction = np.asarray(self._calibration["directions"])
        return ref + direction * (np.array([self._feedback[i].q_rad for i in self.motor_ids]) - mean) / self.config.gear_ratio

    def _soft_ok(self, q: Sequence[float]) -> bool:
        x = np.asarray(q, dtype=float)
        return bool(np.all(x >= self.model.lower - 1e-9) and np.all(x <= self.model.upper + 1e-9))

    def _hard_ok(self, q: Sequence[float]) -> bool:
        x = np.asarray(q, dtype=float)
        return bool(np.all(x >= self.model.hard_lower - 1e-9) and np.all(x <= self.model.hard_upper + 1e-9))

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            self._expire_lease()
            qjoint = self._joint_q()
            joints = []
            for i, mid in enumerate(self.motor_ids):
                f = self._feedback.get(mid)
                joints.append({"name": self.joint_names[i], "motor_id": mid,
                               "q_rotor": None if f is None else f.q_rad,
                               "q_joint": None if not np.isfinite(qjoint[i]) else float(qjoint[i]),
                               "dq_joint": None if f is None else f.dq_rad_s / self.config.gear_ratio,
                               "tau": None if f is None else f.tau_nm,
                               "temperature_c": None if f is None else f.temperature_c,
                               "error": None if f is None else f.error})
            self._sequence += 1
            return {"schema_version": 1, "sequence": self._sequence,
                    "monotonic_ns": int(self._now() * 1e9), "controller_state": self._state,
                    "model_hash": self.model_hash,
                    "calibration_id": None if self._calibration is None else self._calibration["calibration_id"],
                    "board_boot_id": self.board_boot_id, "joints": joints,
                    "lease": None if self._lease_client is None else {"client_id": self._lease_client, "expires_monotonic_ns": int(self._lease_until * 1e9)},
                    "calibration_candidate": self._candidate,
                    "gravity_scale": self._gravity_scale, "active_plan_id": None if self._active_plan is None else self._active_plan["plan_id"],
                    "fault": self._fault, "calibration_valid": self._calibration is not None}

    def _expire_lease(self):
        if self._lease_client is not None and self._now() >= self._lease_until:
            self._lease_client = None
            self._lease_until = 0.0
            if self._state in {"ready", "gravity_hold", "executing", "zero_capture"}:
                self._state = "calibration_valid" if self._calibration else "read_only"
            self._active_plan = None

    def _lease(self, client: str):
        self._expire_lease()
        if self._lease_client != client:
            raise _Reject("lease_required", "control lease is not owned by this client")
        self._lease_until = self._now() + self.config.lease_timeout_s

    def _response(self, req: str, accepted: bool, result: Any = None, error: Any = None):
        return {"schema_version": 1, "request_id": req, "accepted": accepted,
                "result": result, "error": error, "snapshot": self.snapshot()}

    def command(self, name: str, payload: Mapping[str, Any] | None = None, *, request_id: str, client_id: str) -> dict[str, Any]:
        payload = dict(payload or {})
        with self._lock:
            cached = self._responses.get(request_id)
            fingerprint = json.dumps([name, payload, client_id], sort_keys=True, default=str)
            if cached:
                if cached[0] != fingerprint:
                    return self._response(request_id, False, error={"code": "request_id_reuse", "message": "request_id already used"})
                return cached[1]
            try:
                if name not in self._commands:
                    raise _Reject("unknown_command", f"unknown command: {name}")
                result = self._dispatch(name, payload, client_id)
                out = self._response(request_id, True, result=result)
            except _Reject as exc:
                out = self._response(request_id, False, error={"code": exc.code, "message": exc.message})
            self._responses[request_id] = (fingerprint, out)
            return out

    def _dispatch(self, name: str, p: dict[str, Any], client: str) -> dict[str, Any] | None:
        if name == "connection.connect":
            if self._estop: raise _Reject("estop_latched", "reset estop before connect")
            self._state = "read_only" if self._calibration is None else "calibration_valid"; return {"connected": True}
        if name == "connection.disconnect":
            self._lease_client = None; self._active_plan = None; self._state = "disconnected"; return {"connected": False}
        if name == "lease.acquire":
            self._expire_lease()
            if self._lease_client not in (None, client): raise _Reject("lease_busy", "another client owns lease")
            self._lease_client, self._lease_until = client, self._now() + self.config.lease_timeout_s; return {"client_id": client}
        if name == "lease.heartbeat": self._lease(client); return {"expires_monotonic_ns": int(self._lease_until * 1e9)}
        if name == "lease.release":
            self._lease(client); self._lease_client = None; self._active_plan = None
            self._state = "calibration_valid" if self._calibration else "read_only"; return {}
        if name in self._lease_commands: self._lease(client)
        if name in {"control.enable", "gravity.set", "zero.capture", "zero.commit", "plan.validate", "plan.execute"} and self._state == "disconnected": raise _Reject("not_connected", "controller is disconnected")
        if name == "zero.capture":
            if not self._fresh() or not self._healthy(): raise _Reject("feedback_required", "fresh healthy feedback is required")
            ref = _finite_vector(p.get("reference_joint_rad"), self.n, "reference_joint_rad")
            dirs = tuple(int(x) for x in p.get("directions", self.directions_config))
            if len(dirs) != self.n or any(x not in (-1, 1) for x in dirs): raise _Reject("invalid_direction", "directions must be +/-1")
            if not bool(p.get("direction_confirmed", p.get("confirm_direction", False))): raise _Reject("direction_confirmation_required", "confirm motor direction")
            q = self._joint_q() if self._calibration is not None else ref
            if self._calibration is not None and not self._hard_ok(q): raise _Reject("hard_limit", "feedback is outside hard limits")
            self._capture_ref, self._capture_dir, self._capture_values = ref, dirs, []
            self._state = "zero_capture"; return {"capturing": True, "required_samples": self.config.calibration_samples}
        if name == "zero.commit":
            if self._state != "zero_capture" or len(self._capture_values) < self.config.calibration_samples: raise _Reject("capture_incomplete", "200 consecutive stable samples required")
            means = np.mean(np.asarray(self._capture_values), axis=0)
            ref, dirs = self._capture_ref, self._capture_dir
            if not self._hard_ok(ref): raise _Reject("hard_limit", "reference pose exceeds hard limits")
            cid = hashlib.sha256(json.dumps([self.model_hash, self.board_boot_id, means.tolist(), ref.tolist(), dirs], sort_keys=True).encode()).hexdigest()[:16]
            self._candidate = {"calibration_id": cid, "model_hash": self.model_hash, "board_boot_id": self.board_boot_id, "joint_names": list(self.joint_names), "motor_ids": list(self.motor_ids), "directions": list(dirs), "rotor_mean": means.tolist(), "reference_joint_rad": ref.tolist(), "sample_count": len(self._capture_values), "captured_at_utc": datetime.now(timezone.utc).isoformat()}
            self._calibration = dict(self._candidate); self._candidate = self._calibration
            self._state = "calibration_valid"; self._revision += 1; return {"calibration_id": cid, "sample_count": len(self._capture_values)}
        if name == "control.enable":
            enabled = bool(p.get("enabled", True))
            if not enabled: self._active_plan = None; self._state = "calibration_valid" if self._calibration else "read_only"; return {"enabled": False}
            if self._calibration is None or not self._fresh() or not self._healthy(): raise _Reject("enable_guard", "calibration and fresh healthy feedback required")
            q = self._joint_q()
            if not self._soft_ok(q): raise _Reject("soft_limit", "feedback pose is outside soft limits")
            self._targets = q.copy(); self._state = "ready"; return {"enabled": True}
        if name == "gravity.set":
            if self._calibration is None or not self._fresh(): raise _Reject("enable_guard", "calibration and feedback required")
            self._gravity, self._gravity_scale = bool(p.get("enabled", True)), float(p.get("scale", 1.0 if p.get("enabled", True) else 0.0))
            if not 0 <= self._gravity_scale <= 1: raise _Reject("invalid_scale", "gravity scale must be in [0,1]")
            self._state = "gravity_hold" if self._gravity else "ready"; return {"enabled": self._gravity, "scale": self._gravity_scale}
        if name == "stop":
            self._active_plan = None; self._elapsed = 0.0
            if self._state not in {"disconnected", "fault", "estop"}: self._state = "gravity_hold" if self._gravity else ("ready" if self._calibration else "read_only")
            return {"stopped": True}
        if name == "estop": self._estop, self._active_plan, self._state = True, None, "estop"; return {"latched": True}
        if name == "fault.reset":
            if self._state != "fault" and not self._estop: raise _Reject("reset_not_required", "no latched fault")
            self._lease(client); self._estop = False; self._fault = None; self._calibration = None; self._candidate = None; self._active_plan = None; self._state = "read_only" if self._feedback else "disconnected"; return {"reset": True}
        if name == "plan.validate": return self._validate_plan(p)
        if name == "plan.execute":
            plan = self._plans.get(str(p.get("plan_id")))
            if plan is None: raise _Reject("plan_unknown", "plan is unknown")
            if plan["model_hash"] != self.model_hash or self._calibration is None or plan["calibration_id"] != self._calibration["calibration_id"]: raise _Reject("identity_mismatch", "plan identity mismatch")
            if self._state not in {"ready", "gravity_hold"}: raise _Reject("not_ready", "controller is not ready")
            self._active_plan, self._elapsed, self._state = plan, 0.0, "executing"; return {"plan_id": plan["plan_id"]}
        return {}

    def _validate_plan(self, p: Mapping[str, Any]) -> dict[str, Any]:
        if self._calibration is None: raise _Reject("uncalibrated", "calibration required")
        times = np.asarray(p.get("times_s"), dtype=float); pos = np.asarray(p.get("positions_rad"), dtype=float); vel = np.asarray(p.get("velocities_rad_s"), dtype=float)
        if times.ndim != 1 or pos.ndim != 2 or vel.shape != pos.shape or pos.shape[0] != times.size or pos.shape[1] != self.n or times.size < 2 or not np.all(np.isfinite(np.r_[times, pos.flat, vel.flat])): raise _Reject("invalid_plan", "trajectory dimensions or values are invalid")
        if times[0] != 0 or np.any(np.diff(times) <= 0) or np.any(np.diff(times) > .05) or times[-1] > 120: raise _Reject("invalid_plan", "invalid trajectory timing")
        if np.max(np.abs(pos[0] - self._joint_q())) > .02: raise _Reject("start_mismatch", "plan does not start at feedback pose")
        if not self._hard_ok(pos.flatten()) or np.any(pos < self.model.lower) or np.any(pos > self.model.upper): raise _Reject("limit", "trajectory exceeds joint limits")
        if np.max(np.abs(vel)) > self.config.max_velocity_rad_s + 1e-9: raise _Reject("velocity_limit", "trajectory exceeds velocity limit")
        if np.max(np.abs(vel[[0, -1]])) > 1e-7: raise _Reject("endpoint_velocity", "trajectory endpoints must be stopped")
        for k, dt in enumerate(np.diff(times)):
            d = pos[k + 1] - pos[k]; c = vel[k] * dt; e = vel[k + 1] * dt; a, b = c + e - 2*d, 3*d - 2*c - e
            us = [0., 1.]
            for j in range(self.n):
                if abs(a[j]) > 1e-12:
                    disc = max(0., b[j]*b[j]-3*a[j]*c[j]); us += [(-b[j] + math.sqrt(disc))/(3*a[j]), (-b[j]-math.sqrt(disc))/(3*a[j])]
                elif abs(b[j]) > 1e-12: us += [-c[j]/(2*b[j])]
            for u in [x for x in us if 0 <= x <= 1]:
                q = ((a*u+b)*u+c)*u+pos[k]; v = (3*a*u*u+2*b*u+c)/dt; acc = (6*a*u+2*b)/(dt*dt)
                if not self._soft_ok(q) or np.max(np.abs(v)) > self.config.max_velocity_rad_s+1e-7 or np.max(np.abs(acc)) > self.config.max_acceleration_rad_s2+1e-7: raise _Reject("trajectory_limit", "Hermite extrema exceed limits")
            if not self.collision.segment_is_free(pos[k], pos[k+1]): raise _Reject("collision", "trajectory collides with model")
        plan_id = hashlib.sha256(json.dumps({"t":times.tolist(),"p":pos.tolist(),"v":vel.tolist(),"c":self._calibration["calibration_id"]}, sort_keys=True).encode()).hexdigest()[:16]
        plan = {"plan_id": plan_id, "model_hash": self.model_hash, "calibration_id": self._calibration["calibration_id"], "times_s": times.tolist(), "positions_rad": pos.tolist(), "velocities_rad_s": vel.tolist(), "collision_checked": True, "validated": True}
        self._plans[plan_id] = plan; return {"plan": plan}

    def update_feedback(self, frames: Sequence[MotorFeedback | Mapping[str, Any]], now: float | None = None):
        with self._lock:
            timestamp = self._now() if now is None else float(now)
            parsed = {}
            for item in frames:
                f = item if isinstance(item, MotorFeedback) else MotorFeedback(**dict(item))
                if f.motor_id in self.motor_ids: parsed[f.motor_id] = f
            if parsed: self._feedback.update(parsed); self._feedback_time = timestamp
            if self._state == "zero_capture" and self._healthy() and self._fresh(timestamp):
                q = np.array([self._feedback[mid].q_rad for mid in self.motor_ids])
                if all(abs(self._feedback[mid].dq_rad_s) < .02 and self._feedback[mid].error == 0 for mid in self.motor_ids):
                    self._capture_values.append(q)
                else: self._capture_values.clear()
                if len(self._capture_values) > self.config.calibration_samples: self._capture_values = self._capture_values[-self.config.calibration_samples:]

    def report_fault(self, reason: str):
        with self._lock: self._fault, self._active_plan, self._state = str(reason), None, "fault"

    def _hermite(self, plan: dict[str, Any], t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        ts, ps, vs = np.asarray(plan["times_s"]), np.asarray(plan["positions_rad"]), np.asarray(plan["velocities_rad_s"])
        k = min(np.searchsorted(ts, t, side="right") - 1, len(ts)-2); dt = ts[k+1]-ts[k]; u = min(1., max(0., (t-ts[k])/dt)); d=ps[k+1]-ps[k]; c=vs[k]*dt; e=vs[k+1]*dt; a,b=c+e-2*d,3*d-2*c-e
        return ((a*u+b)*u+c)*u+ps[k], (3*a*u*u+2*b*u+c)/dt, (6*a*u+2*b)/(dt*dt)

    def step(self, dt: float) -> tuple[MotorCommand, ...]:
        with self._lock:
            now = self._now(); self._expire_lease()
            if not self._fresh(now) or not self._healthy() or self._state in {"fault", "estop", "disconnected"}: return tuple(MotorCommand(mid, mode="BRAKE") for mid in self.motor_ids)
            q = self._joint_q(); dq = np.array([self._feedback[mid].dq_rad_s for mid in self.motor_ids]) / self.config.gear_ratio
            target, target_v, target_a = q, np.zeros(self.n), np.zeros(self.n)
            if self._active_plan is not None:
                self._elapsed += max(0., float(dt)); target, target_v, target_a = self._hermite(self._active_plan, self._elapsed)
                if self._elapsed >= self._active_plan["times_s"][-1]: self._active_plan = None; self._state = "gravity_hold" if self._gravity else "ready"
            kp, kd = 0.15, 0.02
            tau_joint = kp*(target-q) + kd*(target_v-dq)
            if self._gravity: tau_joint += -self._gravity_scale * self.dynamics.gravity_load(q)
            tau = tau_joint / self.config.gear_ratio; cap = self.config.rotor_torque_cap_nm
            max_step = self.config.torque_slew_nm_s * max(float(dt), 0.)
            tau = np.clip(tau, self._last_tau-max_step, self._last_tau+max_step); tau = np.clip(tau, -cap, cap); self._last_tau = tau
            mean = np.asarray(self._calibration["rotor_mean"]); ref = np.asarray(self._calibration["reference_joint_rad"]); direction = np.asarray(self._calibration["directions"])
            rotor_target = mean + direction*(target-ref)*self.config.gear_ratio
            return tuple(MotorCommand(mid, q_rad=float(rotor_target[i]), dq_rad_s=float(target_v[i]*self.config.gear_ratio), kp=0., kd=0., tau_nm=float(tau[i]), mode="FOC") for i, mid in enumerate(self.motor_ids))

