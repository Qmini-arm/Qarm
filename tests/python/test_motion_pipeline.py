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
    known_goal = np.array([0.20, 0.10, 0.30, 0.10])
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
    sampled_acceleration = np.diff(plan.trajectory.velocities_rad_s, axis=0) / 0.02
    assert np.max(np.abs(sampled_acceleration)) <= 1.0 + 1e-3
    reached = model.fk(plan.trajectory.positions_rad[-1])[:3, 3]
    assert np.linalg.norm(reached - target) <= 0.001


def test_motor_mapping_matches_cpp_conversion_semantics(
    model: ArmModel, mapper: M8010CommandMapper
) -> None:
    start = np.zeros(model.dof)
    q = np.radians([1.0, -2.0, 3.0, -4.0])
    qd = np.array([0.1, -0.1, 0.2, -0.2])
    frame = mapper.map_sample(1.25, q, qd, start)

    assert len(frame.motors) == model.dof
    assert [motor.motor_id for motor in frame.motors] == list(range(model.dof))
    for index, motor in enumerate(frame.motors):
        assert motor.rotor_position_rad is None
        assert np.isclose(motor.rotor_offset_from_start_rad, 6.33 * q[index])
        assert np.isclose(motor.rotor_velocity_rad_s, 6.33 * qd[index])
        assert motor.kp_rotor == 0.2
        assert motor.kd_rotor == 0.03


def test_command_csv_has_one_row_per_joint_per_control_tick(
    tmp_path, model: ArmModel, mapper: M8010CommandMapper
) -> None:
    from qmini_arm_motion.planner import quintic_time_parameterize

    trajectory = quintic_time_parameterize(
        np.vstack([np.zeros(model.dof), np.full(model.dof, 0.05)]),
        np.full(model.dof, 0.5),
        1.0,
        0.02,
    )
    output = mapper.write_csv(trajectory, tmp_path / "commands.csv")
    with output.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == len(trajectory.times_s) * model.dof
    assert {int(row["motor_id"]) for row in rows} == set(range(model.dof))
    assert all(row["rotor_position_rad"] == "" for row in rows)
    sampled_acceleration = np.diff(trajectory.velocities_rad_s, axis=0) / 0.02
    assert np.max(np.abs(sampled_acceleration)) <= 1.0 + 1e-3


def test_plan_home_reaches_urdf_zero_with_smooth_collision_free_trajectory(
    model: ArmModel,
    collision: CollisionChecker,
    mapper: M8010CommandMapper,
) -> None:
    start = np.radians([10.0, 5.0, 10.0, 5.0])
    planner = MotionPlanner(
        model,
        collision,
        config=PlannerConfig(
            velocity_limit_rad_s=mapper.joint_velocity_limit_rad_s,
            acceleration_limit_rad_s2=mapper.joint_acceleration_limit_rad_s2,
            control_period_s=mapper.control_period_s,
        ),
    )

    plan = planner.plan_home(start)

    assert plan.path_kind == "joint_direct"
    assert np.allclose(plan.goal_position_rad, 0.0)
    assert np.allclose(plan.trajectory.positions_rad[0], start)
    assert np.allclose(plan.trajectory.positions_rad[-1], 0.0)
    assert np.allclose(plan.trajectory.velocities_rad_s[[0, -1]], 0.0)
    assert collision.path_is_free(plan.trajectory.positions_rad)
    assert np.all(model.within_limits(plan.trajectory.positions_rad))
    assert np.max(np.abs(plan.trajectory.velocities_rad_s)) <= 0.5 + 1e-8
    sampled_acceleration = np.diff(plan.trajectory.velocities_rad_s, axis=0) / 0.02
    assert np.max(np.abs(sampled_acceleration)) <= 1.0 + 1e-3


def test_plan_to_configuration_rejects_goal_outside_soft_limits(
    model: ArmModel,
    collision: CollisionChecker,
) -> None:
    planner = MotionPlanner(model, collision)
    goal = np.zeros(model.dof)
    goal[0] = model.upper[0] + 0.01

    with np.testing.assert_raises_regex(ValueError, "goal configuration violates"):
        planner.plan_to_configuration(np.zeros(model.dof), goal)


def test_plan_home_rejects_a_self_colliding_start(
    model: ArmModel,
    collision: CollisionChecker,
) -> None:
    hard_model = ArmModel(model.urdf_path, use_soft_limits=False)
    planner = MotionPlanner(hard_model, CollisionChecker(hard_model))
    colliding = np.array([-0.5316577732, 1.1543139485, -2.5678381014, -0.5652933324])

    with np.testing.assert_raises_regex(ValueError, "start configuration self-collides"):
        planner.plan_home(colliding)


def test_plan_home_at_zero_still_returns_a_stopped_trajectory(
    model: ArmModel,
    collision: CollisionChecker,
) -> None:
    plan = MotionPlanner(model, collision).plan_home(np.zeros(model.dof))

    assert len(plan.trajectory.times_s) == 2
    assert np.allclose(plan.trajectory.positions_rad, 0.0)
    assert np.allclose(plan.trajectory.velocities_rad_s, 0.0)


def test_plan_calibration_pose_allows_the_supported_hard_limit_endpoint(
    model: ArmModel,
    collision: CollisionChecker,
) -> None:
    from qarm_sim.calibration_pose import solve_table_supported_pose

    reference = solve_table_supported_pose().joint_position_rad
    start = np.radians([10.0, 5.0, 10.0, 5.0])
    plan = MotionPlanner(
        model,
        collision,
        config=PlannerConfig(
            velocity_limit_rad_s=0.25,
            acceleration_limit_rad_s2=0.50,
            control_period_s=0.01,
        ),
    ).plan_calibration_pose(start, reference)

    assert np.allclose(plan.goal_position_rad, reference)
    assert np.all(model.hard_lower <= plan.trajectory.positions_rad)
    assert np.all(plan.trajectory.positions_rad <= model.hard_upper)
    assert collision.path_is_free(plan.trajectory.positions_rad)
