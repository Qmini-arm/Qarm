from __future__ import annotations

import numpy as np
from qarm_sim.home_simulation import add_endpoint_holds, simulate_home_trajectory
from qarm_sim.model import build_scene
from qmini_arm_motion import CollisionChecker, MotionPlanner, PlannerConfig
from qmini_arm_motion.model import ArmModel


def test_mujoco_return_home_experiment_tracks_bounded_plan(
    model: ArmModel, collision: CollisionChecker
) -> None:
    planner = MotionPlanner(
        model,
        collision,
        config=PlannerConfig(
            velocity_limit_rad_s=0.25,
            acceleration_limit_rad_s2=0.50,
            control_period_s=0.01,
        ),
    )
    plan = planner.plan_home(np.radians([10.0, 5.0, 10.0, 5.0, -5.0, 5.0]))
    trajectory = add_endpoint_holds(
        plan.trajectory,
        control_period_s=0.01,
        start_hold_s=3.0,
        end_hold_s=2.0,
    )

    result = simulate_home_trajectory(build_scene(), trajectory)

    assert result.passed
    assert result.hard_limit_violations == 0
    assert result.contact_steps == 0
    assert result.floor_contact_steps == 0
    assert result.final_floor_contact is False
    assert np.max(np.abs(result.final_error_rad)) <= 0.04
    assert np.all(
        result.maximum_speed_rad_s
        <= np.asarray([0.50, 0.50, 0.50, 0.70, 1.00, 1.50])
    )
    assert result.torque_saturation_steps == 0
