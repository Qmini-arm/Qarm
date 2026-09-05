"""MuJoCo experiment for a collision-checked return-to-zero trajectory."""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np
from numpy.typing import NDArray
from qmini_arm_motion.planner import TimedTrajectory

from qarm_sim.model import ArmScene, set_mirrored_state


@dataclass(frozen=True)
class HomeSimulationConfig:
    gear_ratio: float = 6.33
    kp_rotor: float = 0.20
    kd_rotor: float = 0.03
    rotor_torque_caps_nm: tuple[float, ...] = (
        0.03,
        2.00,
        0.90,
        0.08,
        0.03,
        0.03,
    )
    rotor_torque_slew_nm_per_cycle: float = 2.0 / 256.0
    torque_quantization_nm: float = 1.0 / 256.0
    gain_ramp_s: float = 0.01
    # Conservative output-side regularization for the unidentified reflected
    # rotor/reducer inertia. Zero is known to be numerically non-physical for
    # the tiny wrist inertias; the value is a robustness scenario, not an
    # identified M8010 parameter.
    assumed_joint_armature_kg_m2: float = 0.001
    hard_limit_tolerance_rad: float = 1e-3


@dataclass(frozen=True)
class HomeSimulationResult:
    final_position_rad: NDArray[np.float64]
    final_error_rad: NDArray[np.float64]
    maximum_tracking_error_rad: NDArray[np.float64]
    maximum_speed_rad_s: NDArray[np.float64]
    maximum_abs_joint_torque_nm: NDArray[np.float64]
    torque_saturation_steps: int
    feedforward_slew_steps: int
    hard_limit_violations: int
    contact_steps: int
    floor_contact_steps: int
    final_floor_contact: bool
    finite: bool
    assumed_joint_armature_kg_m2: float
    hard_limit_tolerance_rad: float

    @property
    def passed(self) -> bool:
        return bool(
            self.finite
            and self.hard_limit_violations == 0
            and self.contact_steps == 0
            and np.max(np.abs(self.final_error_rad)) <= 0.04
            and np.all(
                self.maximum_speed_rad_s
                <= np.asarray([0.50, 0.50, 0.50, 0.70, 1.00, 1.50])
            )
            and np.max(self.maximum_tracking_error_rad) <= 0.20
            and self.torque_saturation_steps == 0
        )

    def as_dict(self) -> dict[str, object]:
        return {
            "passed": self.passed,
            "final_position_rad": self.final_position_rad.tolist(),
            "final_error_rad": self.final_error_rad.tolist(),
            "maximum_tracking_error_rad": self.maximum_tracking_error_rad.tolist(),
            "maximum_speed_rad_s": self.maximum_speed_rad_s.tolist(),
            "maximum_abs_joint_torque_nm": self.maximum_abs_joint_torque_nm.tolist(),
            "torque_saturation_steps": self.torque_saturation_steps,
            "feedforward_slew_steps": self.feedforward_slew_steps,
            "hard_limit_violations": self.hard_limit_violations,
            "contact_steps": self.contact_steps,
            "floor_contact_steps": self.floor_contact_steps,
            "final_floor_contact": self.final_floor_contact,
            "finite": self.finite,
            "assumptions": {
                "joint_armature_kg_m2": self.assumed_joint_armature_kg_m2,
                "identified_joint_armature": False,
                "gearbox_friction_backlash_and_delay_included": False,
                "hard_limit_tolerance_rad": self.hard_limit_tolerance_rad,
            },
        }


def add_endpoint_holds(
    trajectory: TimedTrajectory,
    *,
    control_period_s: float,
    start_hold_s: float = 3.0,
    end_hold_s: float = 2.0,
) -> TimedTrajectory:
    """Add stopped dwell time for gain/torque ramp-up and final settling."""

    if control_period_s <= 0.0 or start_hold_s < 0.0 or end_hold_s < 0.0:
        raise ValueError("hold durations must be non-negative and period positive")
    start_count = int(np.ceil(start_hold_s / control_period_s))
    end_count = int(np.ceil(end_hold_s / control_period_s))
    actual_start_hold = start_count * control_period_s
    start_times = np.arange(start_count, dtype=np.float64) * control_period_s
    shifted_times = trajectory.times_s + actual_start_hold
    end_times = shifted_times[-1] + (
        np.arange(1, end_count + 1, dtype=np.float64) * control_period_s
    )
    start_positions = np.repeat(
        trajectory.positions_rad[[0]], start_count, axis=0
    )
    end_positions = np.repeat(
        trajectory.positions_rad[[-1]], end_count, axis=0
    )
    zeros_start = np.zeros_like(start_positions)
    zeros_end = np.zeros_like(end_positions)
    return TimedTrajectory(
        times_s=np.concatenate((start_times, shifted_times, end_times)),
        positions_rad=np.vstack(
            (start_positions, trajectory.positions_rad, end_positions)
        ),
        velocities_rad_s=np.vstack(
            (zeros_start, trajectory.velocities_rad_s, zeros_end)
        ),
    )


def _smoothstep01(value: float) -> float:
    clipped = float(np.clip(value, 0.0, 1.0))
    return clipped * clipped * (3.0 - 2.0 * clipped)


def _interpolate_target(
    trajectory: TimedTrajectory, time_s: float
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    position = np.array(
        [
            np.interp(time_s, trajectory.times_s, trajectory.positions_rad[:, joint])
            for joint in range(trajectory.positions_rad.shape[1])
        ],
        dtype=np.float64,
    )
    velocity = np.array(
        [
            np.interp(time_s, trajectory.times_s, trajectory.velocities_rad_s[:, joint])
            for joint in range(trajectory.velocities_rad_s.shape[1])
        ],
        dtype=np.float64,
    )
    return position, velocity


def simulate_home_trajectory(
    scene: ArmScene,
    trajectory: TimedTrajectory,
    *,
    directions: NDArray[np.float64] | None = None,
    goal_position_rad: NDArray[np.float64] | None = None,
    config: HomeSimulationConfig | None = None,
) -> HomeSimulationResult:
    """Track the planned trajectory with the real controller's idealized law.

    The experiment uses MuJoCo rigid-body dynamics and contacts, rotor-side
    gains, Q8 feed-forward quantization, the deployed feed-forward caps, and
    the deployed per-cycle slew. Floor contact is reported separately because
    it is expected for the supported calibration endpoint; non-floor contact is
    treated as a collision. It intentionally cannot model unidentified gearbox
    friction, backlash, communication delay, or the external cable.
    """

    settings = config or HomeSimulationConfig()
    if settings.assumed_joint_armature_kg_m2 <= 0.0:
        raise ValueError("a positive assumed joint armature is required")
    if settings.hard_limit_tolerance_rad < 0.0:
        raise ValueError("hard-limit tolerance must be non-negative")
    scene.model.dof_armature[scene.dof_addresses] = (
        settings.assumed_joint_armature_kg_m2
    )
    direction = (
        np.ones(6, dtype=np.float64)
        if directions is None
        else np.asarray(directions, dtype=np.float64).reshape(6)
    )
    if np.any(np.abs(direction) != 1.0):
        raise ValueError("directions must contain only +1 or -1")
    caps = np.asarray(settings.rotor_torque_caps_nm, dtype=np.float64)
    if caps.shape != (6,) or np.any(caps <= 0.0):
        raise ValueError("six positive rotor torque caps are required")
    if len(trajectory.times_s) < 2:
        raise ValueError("simulation requires at least two trajectory samples")
    goal = (
        trajectory.positions_rad[-1]
        if goal_position_rad is None
        else np.asarray(goal_position_rad, dtype=np.float64).reshape(6)
    )
    if not np.all(np.isfinite(goal)):
        raise ValueError("goal position must be finite")
    periods = np.diff(trajectory.times_s)
    if np.any(periods <= 0.0):
        raise ValueError("trajectory timestamps must be strictly increasing")
    control_period_s = float(np.min(periods))

    set_mirrored_state(scene, trajectory.positions_rad[0], np.zeros(6))
    # The scene model has real link inertia from URDF and explicit armature
    # only for this conservative simulation scenario. Make the initial state
    # authoritative after changing model parameters.
    mujoco.mj_forward(scene.model, scene.data)
    gravity_data = mujoco.MjData(scene.model)
    previous_feedforward = np.zeros(6, dtype=np.float64)
    maximum_tracking_error = np.zeros(6, dtype=np.float64)
    maximum_speed = np.zeros(6, dtype=np.float64)
    maximum_torque = np.zeros(6, dtype=np.float64)
    torque_saturation_steps = 0
    feedforward_slew_steps = 0
    hard_limit_violations = 0
    contact_steps = 0
    floor_contact_steps = 0
    floor_geom_id = mujoco.mj_name2id(
        scene.model, mujoco.mjtObj.mjOBJ_GEOM, "floor"
    )
    finite = True
    next_control_s = 0.0
    end_time_s = float(trajectory.times_s[-1])
    target_position = trajectory.positions_rad[0].copy()
    target_velocity = np.zeros(6, dtype=np.float64)
    feedforward = np.zeros(6, dtype=np.float64)
    gain = 0.0
    first_control = True

    while scene.data.time < end_time_s - 0.5 * scene.model.opt.timestep:
        if scene.data.time + 1e-12 >= next_control_s:
            target_position, target_velocity = _interpolate_target(
                trajectory, next_control_s
            )
            actual_position = scene.data.qpos[scene.qpos_addresses].copy()

            gravity_data.qpos[scene.qpos_addresses] = actual_position
            gravity_data.qvel[scene.dof_addresses] = 0.0
            mujoco.mj_forward(scene.model, gravity_data)
            gravity_joint = gravity_data.qfrc_bias[scene.dof_addresses].copy()
            requested_feedforward = direction * gravity_joint / settings.gear_ratio
            capped_feedforward = np.clip(requested_feedforward, -caps, caps)
            if first_control:
                # The initial BRAKE feedback already gives us the pose. Seed
                # the feed-forward state so the simulated arm does not fall
                # during a host-side slew from zero; subsequent changes are
                # still slew-limited.
                feedforward = capped_feedforward.copy()
                first_control = False
            else:
                delta = np.clip(
                    capped_feedforward - previous_feedforward,
                    -settings.rotor_torque_slew_nm_per_cycle,
                    settings.rotor_torque_slew_nm_per_cycle,
                )
                feedforward = previous_feedforward + delta
            if not np.allclose(feedforward, requested_feedforward, atol=1e-12):
                feedforward_slew_steps += 1
            quantum = settings.torque_quantization_nm
            feedforward = np.trunc(feedforward / quantum) * quantum
            previous_feedforward = feedforward
            gain = _smoothstep01(next_control_s / settings.gain_ramp_s)
            maximum_tracking_error = np.maximum(
                maximum_tracking_error,
                np.abs(target_position - actual_position),
            )
            next_control_s += control_period_s

        # M8010 closes its rotor PD loop internally at a much higher rate than
        # the host's 100 Hz target updates. Recompute the equivalent joint
        # torque every MuJoCo step; holding a sampled damping torque for 10 ms
        # is numerically and physically the wrong controller.
        actual_position = scene.data.qpos[scene.qpos_addresses]
        actual_velocity = scene.data.qvel[scene.dof_addresses]
        rotor_pd = (
            settings.kp_rotor
            * settings.gear_ratio
            * direction
            * (target_position - actual_position)
            + settings.kd_rotor
            * settings.gear_ratio
            * direction
            * (target_velocity - actual_velocity)
        ) * gain
        requested_joint_torque = direction * settings.gear_ratio * (
            feedforward + rotor_pd
        )
        limited_joint_torque = np.clip(
            requested_joint_torque,
            scene.model.actuator_ctrlrange[:, 0],
            scene.model.actuator_ctrlrange[:, 1],
        )
        if not np.allclose(
            requested_joint_torque, limited_joint_torque, atol=1e-12
        ):
            torque_saturation_steps += 1
        scene.data.ctrl[:] = limited_joint_torque
        maximum_torque = np.maximum(
            maximum_torque, np.abs(limited_joint_torque)
        )
        mujoco.mj_step(scene.model, scene.data)
        position = scene.data.qpos[scene.qpos_addresses]
        velocity = scene.data.qvel[scene.dof_addresses]
        maximum_speed = np.maximum(maximum_speed, np.abs(velocity))
        hard_limit_violations += int(
            np.any(
                position
                < scene.model.jnt_range[scene.joint_ids, 0]
                - settings.hard_limit_tolerance_rad
            )
            or np.any(
                position
                > scene.model.jnt_range[scene.joint_ids, 1]
                + settings.hard_limit_tolerance_rad
            )
        )
        contact_steps += int(
            any(
                scene.data.contact[index].geom1 != floor_geom_id
                and scene.data.contact[index].geom2 != floor_geom_id
                for index in range(scene.data.ncon)
            )
        )
        floor_contact_steps += int(
            any(
                scene.data.contact[index].geom1 == floor_geom_id
                or scene.data.contact[index].geom2 == floor_geom_id
                for index in range(scene.data.ncon)
            )
        )
        finite = finite and bool(
            np.all(np.isfinite(position))
            and np.all(np.isfinite(velocity))
            and np.all(np.isfinite(scene.data.ctrl))
        )
        if not finite:
            break

    final_position = scene.data.qpos[scene.qpos_addresses].copy()
    final_floor_contact = bool(
        any(
            scene.data.contact[index].geom1 == floor_geom_id
            or scene.data.contact[index].geom2 == floor_geom_id
            for index in range(scene.data.ncon)
        )
    )
    return HomeSimulationResult(
        final_position_rad=final_position,
        final_error_rad=goal - final_position,
        maximum_tracking_error_rad=maximum_tracking_error,
        maximum_speed_rad_s=maximum_speed,
        maximum_abs_joint_torque_nm=maximum_torque,
        torque_saturation_steps=torque_saturation_steps,
        feedforward_slew_steps=feedforward_slew_steps,
        hard_limit_violations=hard_limit_violations,
        contact_steps=contact_steps,
        floor_contact_steps=floor_contact_steps,
        final_floor_contact=final_floor_contact,
        finite=finite,
        assumed_joint_armature_kg_m2=(
            settings.assumed_joint_armature_kg_m2
        ),
        hard_limit_tolerance_rad=settings.hard_limit_tolerance_rad,
    )
