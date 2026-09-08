from __future__ import annotations

import numpy as np

from motor_driver import MotorCmd, MotorData, SerialPort
from qarm_sim import (
    JOINT_NAMES,
    QArmMujocoEnv,
    compare_gravity_compensation,
    empirical_gravity_compensation,
    forward_kinematics,
    solve_position_ik,
)


def test_generated_model_is_fixed_four_axis_m8010_scene() -> None:
    with QArmMujocoEnv() as environment:
        info = environment.model_info()

    assert info["fixed_base"] is True
    assert (info["nq"], info["nv"], info["nu"]) == (4, 4, 4)
    assert info["joint_names"] == list(JOINT_NAMES)
    assert info["nsensor"] == 14
    assert np.allclose(info["actuator_ctrlrange_nm"], [-23.7, 23.7])


def test_public_feedforward_torque_maps_directly_to_joint_torque() -> None:
    with QArmMujocoEnv() as environment:
        command = MotorCmd(id=0, mode=1, tau=3.0)
        assert environment.set_command(0, command)
        state = environment.step()

    assert np.isclose(state.requested_torque_nm[0], 3.0)
    assert np.isclose(state.actuator_torque_nm[0], 3.0)


def test_rotor_pd_gain_is_reflected_through_gear_ratio_squared() -> None:
    with QArmMujocoEnv() as environment:
        command = MotorCmd(id=0, mode=1, q=0.1, kp=0.2)
        assert environment.set_command(0, command)
        state = environment.step()

    expected = 0.2 * 6.33**2 * 0.1
    assert np.isclose(state.requested_torque_nm[0], expected)


def test_peak_torque_saturates_and_is_reported() -> None:
    with QArmMujocoEnv() as environment:
        command = MotorCmd(id=0, mode=1, q=1.0, kp=25.0)
        assert environment.set_command(0, command)
        state = environment.step()

        assert environment.saturation_steps == 1
    assert state.requested_torque_nm[0] > 23.7
    assert np.isclose(state.actuator_torque_nm[0], 23.7)


def test_motor_driver_simulation_transport_is_drop_in_and_serial_free() -> None:
    port = SerialPort("mujoco://", realtime=False, initial_qpos=[0.0, 0.2, 0.1, 0.0])
    command = MotorCmd(
        id=2,
        mode=1,
        q=0.2,
        kp=0.2,
        kd=0.03,
        direction=-1,
        offset=1.234,
    )
    feedback = MotorData()
    try:
        assert port.sendRecv(command, feedback)
        state = port.simulation.step(20)
        assert port.sendRecv(command, feedback)
        assert np.isclose(feedback.q, state.position_rad[2])
        assert np.isclose(feedback.dq, state.velocity_rad_s[2])
        assert feedback.temp == 25
        assert feedback.merror == 0
    finally:
        port.close()
    assert not port.serial.is_open


def test_gravity_compensated_hold_stays_near_initial_pose() -> None:
    initial = np.array([0.1, 0.8, 0.2, -0.1])
    with QArmMujocoEnv(initial_qpos=initial) as environment:
        gravity = environment.gravity_compensation()
        for motor_id in range(4):
            command = MotorCmd(
                id=motor_id,
                mode=1,
                q=float(initial[motor_id]),
                tau=float(gravity[motor_id]),
                kp=0.2,
                kd=0.03,
            )
            assert environment.set_command(motor_id, command)
        state = environment.run_for(1.0)

    assert np.max(np.abs(state.position_rad - initial)) < 1e-3
    assert np.all(np.isfinite(state.velocity_rad_s))


def test_inverse_dynamics_gravity_matches_forward_bias() -> None:
    position = np.array([0.1, 0.8, 0.2, -0.1])
    with QArmMujocoEnv(initial_qpos=position) as environment:
        inverse = environment.inverse_dynamics_gravity()
        bias = environment.gravity_compensation()

    assert np.allclose(inverse, bias, atol=1e-10)
    assert np.all(np.isfinite(inverse))


def test_empirical_gravity_formula_keeps_joint_zero_explicit() -> None:
    position = np.array([0.1, 0.8, 0.2, -0.1])
    empirical = empirical_gravity_compensation(position)
    assert empirical.shape == (4,)
    assert empirical[0] == 0.0


def test_gravity_comparison_reports_joint_aligned_errors() -> None:
    position = np.array([0.0, 0.8, 0.2, 0.0])
    with QArmMujocoEnv() as environment:
        comparison = compare_gravity_compensation(environment, position)

    assert comparison.error_nm.shape == (4,)
    assert np.allclose(
        comparison.error_nm,
        comparison.empirical_torque_nm - comparison.mujoco_torque_nm,
    )
    assert comparison.max_absolute_error_nm >= 0.0


def test_fk_and_position_ik_round_trip() -> None:
    source = np.array([0.0, 0.8, 0.2, 0.0])
    with QArmMujocoEnv() as environment:
        pose = forward_kinematics(environment, source)
        result = solve_position_ik(
            environment,
            pose.position_m,
            seed_rad=np.zeros(4),
            position_tolerance_m=1e-5,
        )
        recovered = forward_kinematics(environment, result.position_rad)

    assert result.converged
    assert result.error_norm_m < 1e-5
    assert np.linalg.norm(recovered.position_m - pose.position_m) < 1e-5
    assert np.all(result.position_rad >= np.array([-np.pi, -1.75, -2.62, -2.094395102]))
