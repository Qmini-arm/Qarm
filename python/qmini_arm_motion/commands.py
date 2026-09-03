"""Map joint trajectories to Unitree M8010 rotor-side command fields."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import numpy.typing as npt
import yaml

from .model import ArmModel
from .planner import TimedTrajectory


@dataclass(frozen=True)
class JointCalibration:
    joint_name: str
    motor_id: int
    direction: int
    calibrated: bool
    rotor_zero_rad: float
    joint_zero_rad: float


@dataclass(frozen=True)
class MotorSetpoint:
    joint_name: str
    motor_id: int
    joint_position_rad: float
    joint_velocity_rad_s: float
    rotor_position_rad: float | None
    rotor_offset_from_start_rad: float
    rotor_velocity_rad_s: float
    torque_ff_nm: float
    kp_rotor: float
    kd_rotor: float


@dataclass(frozen=True)
class CommandFrame:
    time_s: float
    motors: tuple[MotorSetpoint, ...]


class M8010CommandMapper:
    """The only Python module that knows motor IDs and reducer coordinates.

    The absolute conversion is identical to the existing C++
    ``jointPositionToRotorRad`` function. Until every joint is calibrated, the
    mapper intentionally returns ``None`` for absolute rotor position; the
    relative offset remains available for simulation and visualization.
    """

    def __init__(
        self,
        model: ArmModel,
        calibrations: tuple[JointCalibration, ...],
        *,
        gear_ratio: float,
        kp_rotor: float,
        kd_rotor: float,
        joint_velocity_limit_rad_s: float,
        joint_acceleration_limit_rad_s2: float,
        control_period_s: float,
    ) -> None:
        if tuple(cal.joint_name for cal in calibrations) != model.joint_names:
            raise ValueError("motor calibration order/names do not match the URDF chain")
        ids = [cal.motor_id for cal in calibrations]
        if len(ids) != 6 or len(set(ids)) != 6 or any(not 0 <= value <= 14 for value in ids):
            raise ValueError("exactly six unique motor IDs in [0, 14] are required")
        if any(cal.direction not in {-1, 1} for cal in calibrations):
            raise ValueError("each motor direction must be +1 or -1")
        if gear_ratio <= 0.0 or kp_rotor < 0.0 or kd_rotor < 0.0:
            raise ValueError("gear ratio must be positive and gains non-negative")
        if (
            joint_velocity_limit_rad_s <= 0.0
            or joint_acceleration_limit_rad_s2 <= 0.0
            or control_period_s <= 0.0
        ):
            raise ValueError("motor trajectory limits and control period must be positive")
        numeric = [
            gear_ratio,
            kp_rotor,
            kd_rotor,
            joint_velocity_limit_rad_s,
            joint_acceleration_limit_rad_s2,
            control_period_s,
        ]
        numeric.extend(cal.rotor_zero_rad for cal in calibrations)
        numeric.extend(cal.joint_zero_rad for cal in calibrations)
        if not np.all(np.isfinite(numeric)):
            raise ValueError("motor configuration values must be finite")
        self.model = model
        self.calibrations = calibrations
        self.gear_ratio = float(gear_ratio)
        self.kp_rotor = float(kp_rotor)
        self.kd_rotor = float(kd_rotor)
        self.joint_velocity_limit_rad_s = float(joint_velocity_limit_rad_s)
        self.joint_acceleration_limit_rad_s2 = float(joint_acceleration_limit_rad_s2)
        self.control_period_s = float(control_period_s)

    @classmethod
    def from_yaml(cls, model: ArmModel, path: str | Path) -> M8010CommandMapper:
        raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        calibrations = tuple(
            JointCalibration(
                joint_name=str(item["name"]),
                motor_id=int(item["motor_id"]),
                direction=int(item["direction"]),
                calibrated=bool(item.get("calibrated", False)),
                rotor_zero_rad=float(item.get("rotor_zero_rad", 0.0)),
                joint_zero_rad=float(item.get("joint_zero_rad", 0.0)),
            )
            for item in raw["joints"]
        )
        return cls(
            model,
            calibrations,
            gear_ratio=float(raw["gear_ratio"]),
            kp_rotor=float(raw["kp_rotor"]),
            kd_rotor=float(raw["kd_rotor"]),
            joint_velocity_limit_rad_s=float(raw["joint_velocity_limit_rad_s"]),
            joint_acceleration_limit_rad_s2=float(raw["joint_acceleration_limit_rad_s2"]),
            control_period_s=float(raw["control_period_s"]),
        )

    @property
    def absolute_positions_available(self) -> bool:
        return all(calibration.calibrated for calibration in self.calibrations)

    def map_sample(
        self,
        time_s: float,
        q: npt.ArrayLike,
        qd: npt.ArrayLike,
        start_q: npt.ArrayLike,
    ) -> CommandFrame:
        positions = np.asarray(q, dtype=np.float64).reshape(self.model.dof)
        velocities = np.asarray(qd, dtype=np.float64).reshape(self.model.dof)
        reference = np.asarray(start_q, dtype=np.float64).reshape(self.model.dof)
        if not self.model.within_limits(positions):
            raise ValueError("commanded joint position violates the URDF soft limits")
        if np.any(np.abs(velocities) > self.joint_velocity_limit_rad_s + 1e-8):
            raise ValueError("commanded joint velocity exceeds the configured M8010 limit")

        motors: list[MotorSetpoint] = []
        for index, calibration in enumerate(self.calibrations):
            direction = float(calibration.direction)
            relative = direction * self.gear_ratio * (positions[index] - reference[index])
            absolute = None
            if calibration.calibrated:
                absolute = calibration.rotor_zero_rad + direction * self.gear_ratio * (
                    positions[index] - calibration.joint_zero_rad
                )
            motors.append(
                MotorSetpoint(
                    joint_name=calibration.joint_name,
                    motor_id=calibration.motor_id,
                    joint_position_rad=float(positions[index]),
                    joint_velocity_rad_s=float(velocities[index]),
                    rotor_position_rad=None if absolute is None else float(absolute),
                    rotor_offset_from_start_rad=float(relative),
                    rotor_velocity_rad_s=float(direction * self.gear_ratio * velocities[index]),
                    torque_ff_nm=0.0,
                    kp_rotor=self.kp_rotor,
                    kd_rotor=self.kd_rotor,
                )
            )
        return CommandFrame(float(time_s), tuple(motors))

    def frames(self, trajectory: TimedTrajectory) -> tuple[CommandFrame, ...]:
        start = trajectory.positions_rad[0]
        return tuple(
            self.map_sample(time_s, q, qd, start)
            for time_s, q, qd in zip(
                trajectory.times_s,
                trajectory.positions_rad,
                trajectory.velocities_rad_s,
                strict=True,
            )
        )

    def write_csv(self, trajectory: TimedTrajectory, output: str | Path) -> Path:
        """Export one row per motor per control tick; this never opens a serial port."""
        destination = Path(output)
        destination.parent.mkdir(parents=True, exist_ok=True)
        with destination.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                [
                    "time_s",
                    "motor_id",
                    "joint_name",
                    "joint_position_rad",
                    "joint_velocity_rad_s",
                    "rotor_position_rad",
                    "rotor_offset_from_start_rad",
                    "rotor_velocity_rad_s",
                    "torque_ff_nm",
                    "kp_rotor",
                    "kd_rotor",
                ]
            )
            for frame in self.frames(trajectory):
                for motor in frame.motors:
                    writer.writerow(
                        [
                            f"{frame.time_s:.6f}",
                            motor.motor_id,
                            motor.joint_name,
                            f"{motor.joint_position_rad:.9f}",
                            f"{motor.joint_velocity_rad_s:.9f}",
                            ""
                            if motor.rotor_position_rad is None
                            else f"{motor.rotor_position_rad:.9f}",
                            f"{motor.rotor_offset_from_start_rad:.9f}",
                            f"{motor.rotor_velocity_rad_s:.9f}",
                            f"{motor.torque_ff_nm:.6f}",
                            f"{motor.kp_rotor:.6f}",
                            f"{motor.kd_rotor:.6f}",
                        ]
                    )
        return destination
