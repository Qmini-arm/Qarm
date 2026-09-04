from __future__ import annotations

import csv

import numpy as np
from qmini_arm_motion import ArmModel, CollisionChecker, M8010CommandMapper
from qmini_arm_motion.ik import IKConfig, PositionIKSolver
from qmini_arm_motion.planner import MotionPlanner, PlannerConfig


def test_collision_free_ik_and_timed_plan(
    model: ArmModel,
    collision: CollisionChecker,
    mapper: M8010CommandMapper,
) -> None:
    start = np.zeros(model.dof)
    known_goal = np.array([0.20, 0.10, 0.30, 0.10, -0.20, 0.10])
    assert collision.is_free(known_goal)
    target = model.fk(known_goal)[:3, 3]
    solver = PositionIKSolver(model, collision, IKConfig(restarts=12, random_seed=4))
    planner = MotionPlanner(
        model,
        collision,
        solver,
        PlannerConfig(
            cartesian_step_m=0.04,
            velocity_limit_rad_s=mapper.joint_velocity_limit_rad_s,
            acceleration_limit_rad_s2=mapper.joint_acceleration_limit_rad_s2,
            control_period_s=mapper.control_period_s,
        ),
    )
    plan = planner.plan(start, target)

    assert plan.ik.success
    assert plan.ik.position_error_m <= 0.001
    assert collision.path_is_free(plan.trajectory.positions_rad)
    assert np.all(model.within_limits(plan.trajectory.positions_rad))
    assert np.max(np.abs(plan.trajectory.velocities_rad_s)) <= 0.5 + 1e-8
    reached = model.fk(plan.trajectory.positions_rad[-1])[:3, 3]
    assert np.linalg.norm(reached - target) <= 0.001


def test_motor_mapping_matches_cpp_conversion_semantics(
    model: ArmModel, mapper: M8010CommandMapper
) -> None:
    start = np.zeros(model.dof)
    q = np.radians([1.0, -2.0, 3.0, -4.0, 5.0, -6.0])
    qd = np.array([0.1, -0.1, 0.2, -0.2, 0.3, -0.3])
    frame = mapper.map_sample(1.25, q, qd, start)

    assert len(frame.motors) == 6
    assert [motor.motor_id for motor in frame.motors] == list(range(6))
    for index, motor in enumerate(frame.motors):
        assert motor.rotor_position_rad is None
        assert np.isclose(motor.rotor_offset_from_start_rad, 6.33 * q[index])
        assert np.isclose(motor.rotor_velocity_rad_s, 6.33 * qd[index])
        assert motor.kp_rotor == 0.2
        assert motor.kd_rotor == 0.03


def test_command_csv_has_six_rows_per_control_tick(
    tmp_path, model: ArmModel, mapper: M8010CommandMapper
) -> None:
    from qmini_arm_motion.planner import quintic_time_parameterize

    trajectory = quintic_time_parameterize(
        np.vstack([np.zeros(6), np.full(6, 0.05)]),
        np.full(6, 0.5),
        1.0,
        0.02,
    )
    output = mapper.write_csv(trajectory, tmp_path / "commands.csv")
    with output.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == len(trajectory.times_s) * 6
    assert {int(row["motor_id"]) for row in rows} == set(range(6))
    assert all(row["rotor_position_rad"] == "" for row in rows)
    sampled_acceleration = np.diff(trajectory.velocities_rad_s, axis=0) / 0.02
    assert np.max(np.abs(sampled_acceleration)) <= 1.0 + 1e-3
