"""URDF-backed joint-space dynamics for the browser simulation.

The implementation intentionally stays independent of Viser and the Unitree
SDK. It uses the same :class:`ArmModel` as FK, IK and collision checking, so
there is only one source for joint axes, link inertias and limits.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt
import yaml

from .commands import CommandFrame
from .model import ArmModel
from .transforms import FloatArray


@dataclass(frozen=True)
class DynamicsConfig:
    """Numerical settings and the gravity vector in the URDF root frame."""

    gravity_root_m_s2: FloatArray
    integration_step_s: float = 0.001
    include_coriolis: bool = False
    mass_regularization_kg_m2: float = 1e-8
    coulomb_smoothing_rad_s: float = 0.01

    def __post_init__(self) -> None:
        gravity = np.asarray(self.gravity_root_m_s2, dtype=np.float64).reshape(3)
        object.__setattr__(self, "gravity_root_m_s2", gravity)
        numeric = (
            self.integration_step_s,
            self.mass_regularization_kg_m2,
            self.coulomb_smoothing_rad_s,
        )
        if not np.all(np.isfinite(gravity)) or not np.all(np.isfinite(numeric)):
            raise ValueError("dynamics configuration must be finite")
        if self.integration_step_s <= 0.0 or self.mass_regularization_kg_m2 < 0.0:
            raise ValueError("integration step must be positive and regularization non-negative")
        if self.coulomb_smoothing_rad_s <= 0.0:
            raise ValueError("Coulomb-friction smoothing speed must be positive")

    @classmethod
    def from_yaml(cls, path: str | Path) -> DynamicsConfig:
        raw: dict[str, Any] = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        simulation = raw.get("simulation")
        if not isinstance(simulation, dict):
            raise ValueError("motor configuration requires a simulation section")
        return cls(
            gravity_root_m_s2=np.asarray(simulation["gravity_root_m_s2"], dtype=np.float64),
            integration_step_s=float(simulation.get("integration_step_s", 0.001)),
            include_coriolis=bool(simulation.get("include_coriolis", False)),
            mass_regularization_kg_m2=float(
                simulation.get("mass_regularization_kg_m2", 1e-8)
            ),
            coulomb_smoothing_rad_s=float(
                simulation.get("coulomb_smoothing_rad_s", 0.01)
            ),
        )


@dataclass(frozen=True)
class DynamicsSample:
    positions_rad: FloatArray
    velocities_rad_s: FloatArray
    accelerations_rad_s2: FloatArray
    control_torque_nm: FloatArray
    gravity_load_nm: FloatArray
    coriolis_torque_nm: FloatArray
    passive_torque_nm: FloatArray


class ArmDynamics:
    """Rigid-link mass, gravity and Coriolis terms in joint coordinates."""

    def __init__(self, model: ArmModel, config: DynamicsConfig) -> None:
        self.model = model
        self.config = config
        self.gravity_base_m_s2 = (
            model.root_to_base[:3, :3].T @ config.gravity_root_m_s2
        )

        inertial_links: list[tuple[str, tuple[int, ...]]] = []
        active: list[int] = []
        joint_indices = {joint.name: index for index, joint in enumerate(model.joints)}
        for joint in model.chain_joints:
            index = joint_indices.get(joint.name)
            if index is not None:
                active.append(index)
            if model.links[joint.child].inertial is not None:
                inertial_links.append((joint.child, tuple(active)))
        if not inertial_links:
            raise ValueError("URDF chain has no moving-link inertial data")
        self._inertial_links = tuple(inertial_links)

    def _link_jacobians(
        self, q: npt.ArrayLike
    ) -> tuple[tuple[FloatArray, FloatArray, FloatArray, float], ...]:
        state = self.model.chain_state(q)
        result: list[tuple[FloatArray, FloatArray, FloatArray, float]] = []
        for link_name, active in self._inertial_links:
            inertial = self.model.links[link_name].inertial
            assert inertial is not None
            inertial_pose = state.link_poses[link_name] @ inertial.origin
            com = inertial_pose[:3, 3]
            linear = np.zeros((3, self.model.dof), dtype=np.float64)
            angular = np.zeros((3, self.model.dof), dtype=np.float64)
            for index in active:
                angular[:, index] = state.axis_directions[index]
                linear[:, index] = np.cross(
                    state.axis_directions[index], com - state.axis_origins[index]
                )
            inertia_base = (
                inertial_pose[:3, :3]
                @ inertial.inertia_kg_m2
                @ inertial_pose[:3, :3].T
            )
            result.append((linear, angular, inertia_base, inertial.mass_kg))
        return tuple(result)

    def mass_matrix(self, q: npt.ArrayLike) -> FloatArray:
        matrix = np.zeros((self.model.dof, self.model.dof), dtype=np.float64)
        for linear, angular, inertia_base, mass in self._link_jacobians(q):
            matrix += mass * (linear.T @ linear) + angular.T @ inertia_base @ angular
        matrix += np.eye(self.model.dof) * self.config.mass_regularization_kg_m2
        return 0.5 * (matrix + matrix.T)

    def gravity_load(self, q: npt.ArrayLike) -> FloatArray:
        """Return generalized torque exerted by gravity, not compensation torque."""
        load = np.zeros(self.model.dof, dtype=np.float64)
        for linear, _angular, _inertia_base, mass in self._link_jacobians(q):
            load += linear.T @ (mass * self.gravity_base_m_s2)
        return load

    def potential_energy(self, q: npt.ArrayLike) -> float:
        state = self.model.chain_state(q)
        energy = 0.0
        for link_name, _active in self._inertial_links:
            inertial = self.model.links[link_name].inertial
            assert inertial is not None
            com = (state.link_poses[link_name] @ inertial.origin)[:3, 3]
            energy -= inertial.mass_kg * float(self.gravity_base_m_s2 @ com)
        return energy

    def coriolis_torque(self, q: npt.ArrayLike, qd: npt.ArrayLike) -> FloatArray:
        velocity = np.asarray(qd, dtype=np.float64).reshape(self.model.dof)
        if not self.config.include_coriolis or np.max(np.abs(velocity)) <= 1e-12:
            return np.zeros(self.model.dof, dtype=np.float64)
        positions = np.asarray(q, dtype=np.float64).reshape(self.model.dof)
        derivative = np.empty((self.model.dof, self.model.dof, self.model.dof))
        step = 1e-5
        for coordinate in range(self.model.dof):
            offset = np.zeros(self.model.dof)
            offset[coordinate] = step
            derivative[coordinate] = (
                self.mass_matrix(positions + offset) - self.mass_matrix(positions - offset)
            ) / (2.0 * step)

        torque = np.zeros(self.model.dof, dtype=np.float64)
        for i in range(self.model.dof):
            for j in range(self.model.dof):
                for k in range(self.model.dof):
                    christoffel = 0.5 * (
                        derivative[k, i, j]
                        + derivative[j, i, k]
                        - derivative[i, j, k]
                    )
                    torque[i] += christoffel * velocity[j] * velocity[k]
        return torque

    def passive_torque(self, qd: npt.ArrayLike) -> FloatArray:
        velocity = np.asarray(qd, dtype=np.float64).reshape(self.model.dof)
        return self.model.damping * velocity + self.model.friction * np.tanh(
            velocity / self.config.coulomb_smoothing_rad_s
        )

    def acceleration(
        self,
        q: npt.ArrayLike,
        qd: npt.ArrayLike,
        control_torque_nm: npt.ArrayLike,
    ) -> tuple[FloatArray, FloatArray, FloatArray, FloatArray]:
        positions = np.asarray(q, dtype=np.float64).reshape(self.model.dof)
        velocity = np.asarray(qd, dtype=np.float64).reshape(self.model.dof)
        control = np.asarray(control_torque_nm, dtype=np.float64).reshape(self.model.dof)
        gravity = self.gravity_load(positions)
        coriolis = self.coriolis_torque(positions, velocity)
        passive = self.passive_torque(velocity)
        acceleration = np.linalg.solve(
            self.mass_matrix(positions), control + gravity - coriolis - passive
        )
        return acceleration, gravity, coriolis, passive


class MotorDynamicsSimulator:
    """Integrate the M8010 hybrid PD command against the URDF dynamics."""

    def __init__(
        self,
        dynamics: ArmDynamics,
        initial_q: npt.ArrayLike,
        *,
        gear_ratio: float,
        directions: npt.ArrayLike,
    ) -> None:
        self.dynamics = dynamics
        self.model = dynamics.model
        if gear_ratio <= 0.0:
            raise ValueError("simulation gear ratio must be positive")
        values = np.asarray(directions, dtype=np.float64).reshape(self.model.dof)
        if np.any(np.abs(values) != 1.0):
            raise ValueError("simulation directions must be +1 or -1")
        self.gear_ratio = float(gear_ratio)
        self.directions = values
        self.reset(initial_q)

    def reset(self, q: npt.ArrayLike) -> None:
        positions = np.asarray(q, dtype=np.float64).reshape(self.model.dof)
        if not self.model.within_limits(positions):
            raise ValueError("simulator reset position violates the URDF soft limits")
        self.q = positions.copy()
        self.qd = np.zeros(self.model.dof, dtype=np.float64)

    def _actuator_terms(
        self, frame: CommandFrame
    ) -> tuple[FloatArray, FloatArray, FloatArray, FloatArray, FloatArray]:
        if tuple(motor.joint_name for motor in frame.motors) != self.model.joint_names:
            raise ValueError("simulation command order does not match the URDF chain")
        target = np.empty(self.model.dof, dtype=np.float64)
        target_velocity = np.empty(self.model.dof, dtype=np.float64)
        feedforward = np.empty(self.model.dof, dtype=np.float64)
        kp = np.empty(self.model.dof, dtype=np.float64)
        kd = np.empty(self.model.dof, dtype=np.float64)
        for index, motor in enumerate(frame.motors):
            # Position/velocity commands and gains are rotor-side. Direction
            # cancels from the PD terms after ideal reducer conversion.
            target[index] = motor.joint_position_rad
            target_velocity[index] = motor.joint_velocity_rad_s
            feedforward[index] = (
                self.directions[index] * self.gear_ratio * motor.torque_ff_nm
            )
            kp[index] = motor.kp_rotor * self.gear_ratio * self.gear_ratio
            kd[index] = motor.kd_rotor * self.gear_ratio * self.gear_ratio
        return target, target_velocity, feedforward, kp, kd

    def advance(
        self,
        frame: CommandFrame,
        duration_s: float,
        *,
        compensate_gravity: bool = False,
    ) -> DynamicsSample:
        if duration_s < 0.0 or not np.isfinite(duration_s):
            raise ValueError("simulation duration must be finite and non-negative")
        steps = max(1, int(np.ceil(duration_s / self.dynamics.config.integration_step_s)))
        step = duration_s / steps
        target, target_velocity, feedforward, kp, kd = self._actuator_terms(frame)
        control = np.zeros(self.model.dof)
        acceleration = np.zeros(self.model.dof)
        gravity = np.zeros(self.model.dof)
        coriolis = np.zeros(self.model.dof)
        passive = np.zeros(self.model.dof)
        for _ in range(steps):
            matrix = self.dynamics.mass_matrix(self.q)
            gravity = self.dynamics.gravity_load(self.q)
            coriolis = self.dynamics.coriolis_torque(self.q, self.qd)
            coulomb = self.model.friction * np.tanh(
                self.qd / self.dynamics.config.coulomb_smoothing_rad_s
            )
            previous_velocity = self.qd.copy()
            actuator_constant = (
                feedforward
                + kp * (target - self.q)
                + kd * target_velocity
                - gravity * float(compensate_gravity)
            )
            saturation = np.zeros(self.model.dof, dtype=np.int8)
            next_velocity = previous_velocity
            for _active_set_iteration in range(self.model.dof + 2):
                unsaturated = saturation == 0
                damping = self.model.damping + np.where(unsaturated, kd, 0.0)
                applied_constant = np.where(
                    unsaturated, actuator_constant, saturation * self.model.effort
                )
                lhs = matrix + step * np.diag(damping)
                rhs = matrix @ previous_velocity + step * (
                    applied_constant + gravity - coriolis - coulomb
                )
                next_velocity = np.linalg.solve(lhs, rhs)
                raw_control = actuator_constant - kd * next_velocity
                next_saturation = np.where(
                    raw_control > self.model.effort,
                    1,
                    np.where(raw_control < -self.model.effort, -1, 0),
                ).astype(np.int8)
                if np.array_equal(next_saturation, saturation):
                    break
                saturation = next_saturation
            self.qd = next_velocity
            acceleration = (self.qd - previous_velocity) / max(step, 1e-12)
            self.qd = np.clip(self.qd, -self.model.velocity, self.model.velocity)
            self.q += step * self.qd
            below = self.q < self.model.hard_lower
            above = self.q > self.model.hard_upper
            self.q = np.clip(self.q, self.model.hard_lower, self.model.hard_upper)
            self.qd[below & (self.qd < 0.0)] = 0.0
            self.qd[above & (self.qd > 0.0)] = 0.0
            control = np.clip(
                actuator_constant - kd * self.qd,
                -self.model.effort,
                self.model.effort,
            )
            passive = self.dynamics.passive_torque(self.qd)
        return DynamicsSample(
            self.q.copy(),
            self.qd.copy(),
            acceleration.copy(),
            control.copy(),
            gravity.copy(),
            coriolis.copy(),
            passive.copy(),
        )
