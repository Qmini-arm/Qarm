"""Four-axis Qarm rigid-body simulation with an M8010 command adapter.

The public command coordinates intentionally match :mod:`motor_driver`:

* ``q`` and ``dq`` are output-joint position and velocity;
* ``tau`` is ideal output-joint feed-forward torque;
* ``kp`` and ``kd`` are the rotor-side gains encoded in the M8010 packet.

Consequently, an ideal gearbox reflects the position and velocity gains to the
joint by ``gear_ratio ** 2``.  Direction and encoder zero offset disappear at
this boundary because ``motor_driver.SerialPort.sendRecv`` exposes calibrated
joint coordinates on both command and feedback.
"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL_PATH = ROOT / "description" / "output_mjcf" / "robot.xml"
JOINT_NAMES = ("joint_1", "joint_2", "joint_3", "joint_4")


FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class M8010Parameters:
    """Known public limits plus explicit, uncalibrated simulation assumptions."""

    model: str = "GO-M8010-6"
    gear_ratio: float = 6.33
    peak_output_torque_nm: float = 23.7
    maximum_output_speed_rad_s: float = 30.0
    arm_joint_speed_limit_rad_s: float = 3.0
    rotor_gain_limit: float = 25.599
    ambient_temperature_c: int = 25
    source: str = "https://www.unitree.com/cn/go1motor"

    def __post_init__(self) -> None:
        positive = (
            self.gear_ratio,
            self.peak_output_torque_nm,
            self.maximum_output_speed_rad_s,
            self.arm_joint_speed_limit_rad_s,
            self.rotor_gain_limit,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("M8010 parameters must be finite and positive")


@dataclass(frozen=True)
class MotorFeedback:
    q: float
    dq: float
    tau: float
    temp: int
    merror: int = 0


@dataclass(frozen=True)
class SimulationState:
    time_s: float
    position_rad: FloatArray
    velocity_rad_s: FloatArray
    actuator_torque_nm: FloatArray
    requested_torque_nm: FloatArray
    command_clipped: NDArray[np.bool_]
    tool_position_m: FloatArray
    contacts: tuple[tuple[str, str], ...]


@dataclass
class _Command:
    mode: int = 0
    q: float = 0.0
    dq: float = 0.0
    tau: float = 0.0
    kp: float = 0.0
    kd: float = 0.0


class QArmMujocoEnv:
    """MuJoCo environment for the fixed-base, four-axis Qarm.

    Use :meth:`step` for deterministic/headless experiments, or
    :meth:`start_realtime` when using the drop-in ``SerialPort`` interface.
    The latter mirrors a physical bus: physics advances continuously while
    ``send_recv`` only exchanges the newest command and feedback.
    """

    def __init__(
        self,
        model_path: str | Path = DEFAULT_MODEL_PATH,
        *,
        parameters: M8010Parameters | None = None,
        initial_qpos: Iterable[float] | None = None,
        realtime_factor: float = 1.0,
    ) -> None:
        self.model_path = Path(model_path).expanduser().resolve()
        if not self.model_path.is_file():
            raise FileNotFoundError(f"MuJoCo model not found: {self.model_path}")
        self.parameters = parameters or M8010Parameters()
        if not math.isfinite(realtime_factor) or realtime_factor <= 0.0:
            raise ValueError("realtime_factor must be finite and positive")
        self.realtime_factor = float(realtime_factor)

        try:
            self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        except ValueError as error:
            raise ValueError(f"cannot compile MuJoCo model {self.model_path}: {error}") from error
        self.data = mujoco.MjData(self.model)
        self._bias_data = mujoco.MjData(self.model)
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._closed = False

        self.joint_ids = np.asarray(
            [
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                for name in JOINT_NAMES
            ],
            dtype=np.int32,
        )
        if np.any(self.joint_ids < 0):
            missing = [
                name for name, joint_id in zip(JOINT_NAMES, self.joint_ids) if joint_id < 0
            ]
            raise ValueError(f"MuJoCo model is missing arm joints: {missing}")
        if self.model.nq != len(JOINT_NAMES) or self.model.nv != len(JOINT_NAMES):
            raise ValueError(
                "Qarm simulation requires a fixed base and exactly four hinge DOFs; "
                f"model has nq={self.model.nq}, nv={self.model.nv}"
            )
        if np.any(self.model.jnt_type[self.joint_ids] != mujoco.mjtJoint.mjJNT_HINGE):
            raise ValueError("joint_1 through joint_4 must all be hinge joints")

        self.qpos_addresses = self.model.jnt_qposadr[self.joint_ids].copy()
        self.dof_addresses = self.model.jnt_dofadr[self.joint_ids].copy()
        self.actuator_ids = self._find_actuators()
        self.tool_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "tool0_site"
        )
        if self.tool_site_id < 0:
            raise ValueError("MuJoCo model must define site 'tool0_site'")

        self._commands = [_Command() for _ in JOINT_NAMES]
        self._requested_torque = np.zeros(len(JOINT_NAMES), dtype=np.float64)
        self._command_clipped = np.zeros(len(JOINT_NAMES), dtype=np.bool_)
        self._saturation_steps = 0
        self.reset(initial_qpos)

    def _find_actuators(self) -> NDArray[np.int32]:
        if self.model.nu != len(JOINT_NAMES):
            raise ValueError(f"expected four M8010 actuators, found {self.model.nu}")
        result = np.full(len(JOINT_NAMES), -1, dtype=np.int32)
        for joint_index, joint_id in enumerate(self.joint_ids):
            matches = np.flatnonzero(self.model.actuator_trnid[:, 0] == joint_id)
            if len(matches) != 1:
                raise ValueError(
                    f"{JOINT_NAMES[joint_index]} must have exactly one direct actuator"
                )
            result[joint_index] = int(matches[0])
        if not np.all(self.model.actuator_ctrllimited[result]):
            raise ValueError("all M8010 actuators must define ctrlrange")
        ranges = self.model.actuator_ctrlrange[result]
        expected = self.parameters.peak_output_torque_nm
        if np.any(ranges[:, 0] > -expected) or np.any(ranges[:, 1] < expected):
            raise ValueError(
                "actuator ctrlrange is narrower than the configured M8010 peak torque"
            )
        return result

    @property
    def timestep(self) -> float:
        return float(self.model.opt.timestep)

    @property
    def is_open(self) -> bool:
        return not self._closed

    @property
    def is_realtime(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def saturation_steps(self) -> int:
        with self._lock:
            return self._saturation_steps

    def reset(self, qpos: Iterable[float] | None = None) -> SimulationState:
        position = np.zeros(len(JOINT_NAMES), dtype=np.float64)
        if qpos is not None:
            position = np.asarray(tuple(qpos), dtype=np.float64)
        if position.shape != (len(JOINT_NAMES),) or not np.all(np.isfinite(position)):
            raise ValueError("initial_qpos must contain four finite joint angles")
        ranges = self.model.jnt_range[self.joint_ids]
        if np.any(position < ranges[:, 0]) or np.any(position > ranges[:, 1]):
            raise ValueError("initial_qpos violates a MuJoCo joint range")

        with self._lock:
            self._require_open()
            mujoco.mj_resetData(self.model, self.data)
            self.data.qpos[self.qpos_addresses] = position
            self.data.qvel[self.dof_addresses] = 0.0
            self.data.ctrl[self.actuator_ids] = 0.0
            for index, value in enumerate(position):
                self._commands[index] = _Command(q=float(value))
            self._requested_torque.fill(0.0)
            self._command_clipped.fill(False)
            self._saturation_steps = 0
            mujoco.mj_forward(self.model, self.data)
            return self._snapshot_unlocked()

    def set_command(self, motor_id: int, command: Any) -> bool:
        """Copy one ``motor_driver.MotorCmd``-shaped object into the controller."""

        if self._closed or motor_id < 0 or motor_id >= len(JOINT_NAMES):
            return False
        try:
            mode = int(command.mode)
            values = np.asarray(
                [command.q, command.dq, command.tau, command.kp, command.kd],
                dtype=np.float64,
            )
            direction = int(command.direction)
        except (AttributeError, TypeError, ValueError):
            return False
        if mode not in (0, 1) or direction not in (-1, 1) or not np.all(np.isfinite(values)):
            return False

        q, dq, tau, kp, kd = (float(value) for value in values)
        gain_limit = self.parameters.rotor_gain_limit
        speed_limit = min(
            self.parameters.maximum_output_speed_rad_s,
            self.parameters.arm_joint_speed_limit_rad_s,
        )
        joint_range = self.model.jnt_range[self.joint_ids[motor_id]]
        clipped_q = float(np.clip(q, joint_range[0], joint_range[1]))
        clipped_dq = float(np.clip(dq, -speed_limit, speed_limit))
        clipped_tau = float(
            np.clip(
                tau,
                -self.parameters.peak_output_torque_nm,
                self.parameters.peak_output_torque_nm,
            )
        )
        clipped_kp = float(np.clip(kp, 0.0, gain_limit))
        clipped_kd = float(np.clip(kd, 0.0, gain_limit))
        clipped = not np.allclose(
            [q, dq, tau, kp, kd],
            [clipped_q, clipped_dq, clipped_tau, clipped_kp, clipped_kd],
            rtol=0.0,
            atol=0.0,
        )
        with self._lock:
            self._commands[motor_id] = _Command(
                mode=mode,
                q=clipped_q,
                dq=clipped_dq,
                tau=clipped_tau,
                kp=clipped_kp,
                kd=clipped_kd,
            )
            self._command_clipped[motor_id] = clipped
        return True

    def send_recv(self, command: Any, feedback: Any) -> bool:
        """Drop-in implementation of ``SerialPort.sendRecv``.

        This method does not advance simulation time.  Start the real-time loop
        for legacy code, or call :meth:`step` explicitly in deterministic code.
        """

        try:
            motor_id = int(command.id)
        except (AttributeError, TypeError, ValueError):
            return False
        if not self.set_command(motor_id, command):
            if hasattr(feedback, "merror"):
                feedback.merror = 1
            return False
        sample = self.read_motor(motor_id)
        feedback.q = sample.q
        feedback.dq = sample.dq
        feedback.tau = sample.tau
        feedback.temp = sample.temp
        feedback.merror = sample.merror
        return True

    # Keep the exact spelling used by the original motor_driver.SerialPort.
    def sendRecv(self, command: Any, feedback: Any) -> bool:
        return self.send_recv(command, feedback)

    def read_motor(self, motor_id: int) -> MotorFeedback:
        if motor_id < 0 or motor_id >= len(JOINT_NAMES):
            raise ValueError(f"motor_id must be in 0..{len(JOINT_NAMES) - 1}")
        with self._lock:
            self._require_open()
            return MotorFeedback(
                q=float(self.data.qpos[self.qpos_addresses[motor_id]]),
                dq=float(self.data.qvel[self.dof_addresses[motor_id]]),
                tau=float(self.data.actuator_force[self.actuator_ids[motor_id]]),
                # Thermal and controller fault models require hardware identification.
                temp=self.parameters.ambient_temperature_c,
                merror=0,
            )

    def inverse_dynamics_gravity(
        self, position_rad: Iterable[float] | None = None
    ) -> FloatArray:
        """Return static gravity torque from MuJoCo inverse dynamics.

        MuJoCo's ``mj_forward`` fills ``qacc`` with the acceleration produced
        by the current forces.  For inverse dynamics we therefore call it
        first, overwrite ``qacc`` with the desired zero acceleration, and then
        call ``mj_inverse``.  At zero velocity and acceleration,
        ``qfrc_inverse`` is the generalized torque required to balance gravity.
        """

        with self._lock:
            self._require_open()
            if position_rad is None:
                position = self.data.qpos.copy()
            else:
                position = np.asarray(tuple(position_rad), dtype=np.float64)
                if position.shape != (len(JOINT_NAMES),) or not np.all(np.isfinite(position)):
                    raise ValueError("position_rad must contain four finite joint angles")
                ranges = self.model.jnt_range[self.joint_ids]
                if np.any(position < ranges[:, 0]) or np.any(position > ranges[:, 1]):
                    raise ValueError("position_rad violates a MuJoCo joint range")

            self._bias_data.qpos[:] = position
            self._bias_data.qvel[:] = 0.0
            self._bias_data.ctrl[:] = 0.0
            self._bias_data.qfrc_applied[:] = 0.0
            mujoco.mj_forward(self.model, self._bias_data)
            self._bias_data.qacc[:] = 0.0
            mujoco.mj_inverse(self.model, self._bias_data)
            return self._bias_data.qfrc_inverse[self.dof_addresses].copy()

    def gravity_compensation(self) -> FloatArray:
        """Return ideal output-joint torque that balances gravity at current q."""

        return self.inverse_dynamics_gravity()

    def _control_unlocked(self) -> None:
        q = self.data.qpos[self.qpos_addresses]
        dq = self.data.qvel[self.dof_addresses]
        ratio_squared = self.parameters.gear_ratio**2
        requested = np.zeros(len(JOINT_NAMES), dtype=np.float64)
        for index, command in enumerate(self._commands):
            if command.mode == 1:
                requested[index] = (
                    command.tau
                    + ratio_squared * command.kp * (command.q - q[index])
                    + ratio_squared * command.kd * (command.dq - dq[index])
                )
            # mode 0 means no electrical torque in this uncalibrated model.

        lower = np.maximum(
            self.model.actuator_ctrlrange[self.actuator_ids, 0],
            -self.parameters.peak_output_torque_nm,
        )
        upper = np.minimum(
            self.model.actuator_ctrlrange[self.actuator_ids, 1],
            self.parameters.peak_output_torque_nm,
        )
        applied = np.clip(requested, lower, upper)
        self._saturation_steps += int(not np.allclose(requested, applied, atol=1e-12))
        self._requested_torque[:] = requested
        self.data.ctrl[self.actuator_ids] = applied

    def step(self, n_steps: int = 1) -> SimulationState:
        if isinstance(n_steps, bool) or not isinstance(n_steps, int) or n_steps <= 0:
            raise ValueError("n_steps must be a positive integer")
        if self.is_realtime:
            raise RuntimeError("explicit step is unavailable while the real-time loop is running")
        with self._lock:
            self._require_open()
            for _ in range(n_steps):
                self._control_unlocked()
                mujoco.mj_step(self.model, self.data)
            return self._snapshot_unlocked()

    def run_for(self, duration_s: float) -> SimulationState:
        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ValueError("duration_s must be finite and positive")
        return self.step(max(1, math.ceil(duration_s / self.timestep)))

    def snapshot(self) -> SimulationState:
        with self._lock:
            self._require_open()
            return self._snapshot_unlocked()

    def _snapshot_unlocked(self) -> SimulationState:
        contacts: list[tuple[str, str]] = []
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            first = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1
            ) or f"geom_{contact.geom1}"
            second = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2
            ) or f"geom_{contact.geom2}"
            contacts.append((first, second))
        return SimulationState(
            time_s=float(self.data.time),
            position_rad=self.data.qpos[self.qpos_addresses].copy(),
            velocity_rad_s=self.data.qvel[self.dof_addresses].copy(),
            actuator_torque_nm=self.data.actuator_force[self.actuator_ids].copy(),
            requested_torque_nm=self._requested_torque.copy(),
            command_clipped=self._command_clipped.copy(),
            tool_position_m=self.data.site_xpos[self.tool_site_id].copy(),
            contacts=tuple(contacts),
        )

    def start_realtime(self) -> None:
        with self._lock:
            self._require_open()
            if self.is_realtime:
                return
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._realtime_loop,
                name="qarm-mujoco-physics",
                daemon=True,
            )
            self._thread.start()

    def _realtime_loop(self) -> None:
        period = self.timestep / self.realtime_factor
        next_step = time.perf_counter()
        while not self._stop_event.is_set():
            now = time.perf_counter()
            if now < next_step:
                self._stop_event.wait(min(next_step - now, period))
                continue
            due = int((now - next_step) / period) + 1
            # Bound catch-up so a paused process does not spend seconds racing ahead.
            due = min(due, 20)
            with self._lock:
                if self._closed:
                    return
                for _ in range(due):
                    self._control_unlocked()
                    mujoco.mj_step(self.model, self.data)
            next_step += due * period
            if time.perf_counter() - next_step > 20 * period:
                next_step = time.perf_counter() + period

    def stop_realtime(self) -> None:
        thread = self._thread
        if thread is None:
            return
        self._stop_event.set()
        if thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self._thread = None

    def launch_viewer(self, *, duration_s: float = 0.0) -> None:
        """Open MuJoCo's passive viewer and keep physics synchronized."""

        if duration_s < 0.0 or not math.isfinite(duration_s):
            raise ValueError("duration_s must be finite and non-negative")
        import mujoco.viewer

        started_here = not self.is_realtime
        if started_here:
            self.start_realtime()
        started = time.monotonic()
        try:
            with mujoco.viewer.launch_passive(
                self.model, self.data, show_left_ui=False, show_right_ui=True
            ) as viewer:
                while viewer.is_running():
                    if duration_s > 0.0 and time.monotonic() - started >= duration_s:
                        break
                    with self._lock:
                        viewer.sync()
                    time.sleep(1.0 / 60.0)
        finally:
            if started_here:
                self.stop_realtime()

    def model_info(self) -> dict[str, Any]:
        ranges = self.model.jnt_range[self.joint_ids]
        return {
            "model_path": str(self.model_path),
            "mujoco_version": mujoco.__version__,
            "fixed_base": self.model.nq == len(JOINT_NAMES),
            "nq": self.model.nq,
            "nv": self.model.nv,
            "nu": self.model.nu,
            "nbody": self.model.nbody,
            "ngeom": self.model.ngeom,
            "nmesh": self.model.nmesh,
            "nsensor": self.model.nsensor,
            "timestep_s": self.timestep,
            "joint_names": list(JOINT_NAMES),
            "joint_range_rad": ranges.tolist(),
            "actuator_ctrlrange_nm": self.model.actuator_ctrlrange[
                self.actuator_ids
            ].tolist(),
            "motor": {
                "model": self.parameters.model,
                "gear_ratio": self.parameters.gear_ratio,
                "peak_output_torque_nm": self.parameters.peak_output_torque_nm,
                "maximum_output_speed_rad_s": (
                    self.parameters.maximum_output_speed_rad_s
                ),
                "source": self.parameters.source,
            },
            "unidentified": [
                "rotor and reflected gearbox inertia",
                "continuous torque and thermal limits",
                "Coulomb friction and viscous damping",
                "backlash and gearbox efficiency",
                "command/feedback delay and packet loss",
                "mode-0 electrical braking behavior",
            ],
        }

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("QArmMujocoEnv is closed")

    def close(self) -> None:
        if self._closed:
            return
        self.stop_realtime()
        with self._lock:
            self._closed = True

    def __enter__(self):
        self._require_open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
