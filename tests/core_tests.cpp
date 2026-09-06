#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "qmini_arm/joint_conversion.hpp"
#include "qmini_arm/gravity_model.hpp"
#include "qmini_arm/gravity_control.hpp"
#include "qmini_arm/joint_trajectory.hpp"
#include "qmini_arm/safety.hpp"
#include "qmini_arm/sine_trajectory.hpp"

namespace {

bool near(double actual, double expected, double tolerance = 1e-9) {
  return std::abs(actual - expected) <= tolerance;
}

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

template <typename Callable>
void requireThrows(Callable callable, const std::string& message) {
  try {
    callable();
  } catch (const std::exception&) {
    return;
  }
  throw std::runtime_error(message);
}

void testJointConversion() {
  qmini_arm::MotorState motor;
  motor.motor_id = 3;
  motor.position_rad = 16.33;
  motor.velocity_rad_s = 12.66;
  motor.torque_estimate_nm = 1.0;

  qmini_arm::JointCalibration calibration;
  calibration.motor_id = 3;
  calibration.direction = 1;
  calibration.gear_ratio = 6.33;
  calibration.rotor_zero_rad = 10.0;
  calibration.joint_zero_rad = 0.25;
  calibration.position_calibrated = true;

  const qmini_arm::JointState joint =
      qmini_arm::toJointState(motor, calibration);
  require(joint.position_calibrated, "calibrated flag was lost");
  require(near(joint.position_rad, 1.25), "joint position conversion failed");
  require(near(joint.velocity_rad_s, 2.0),
          "joint velocity conversion failed");
  require(near(joint.torque_estimate_nm, 6.33),
          "joint torque conversion failed");
  require(near(qmini_arm::jointPositionToRotorRad(1.25, calibration), 16.33),
          "inverse position conversion failed");
  require(near(qmini_arm::jointVelocityToRotorRadS(2.0, calibration), 12.66),
          "inverse velocity conversion failed");
  require(near(qmini_arm::jointTorqueToRotorNm(6.33, calibration), 1.0),
          "inverse torque conversion failed");
  require(near(qmini_arm::jointKpToRotor(6.33 * 6.33, calibration), 1.0),
          "kp conversion failed");

  calibration.position_calibrated = false;
  const qmini_arm::JointState uncalibrated =
      qmini_arm::toJointState(motor, calibration);
  require(std::isnan(uncalibrated.position_rad),
          "uncalibrated joint position must be NaN");
  requireThrows(
      [&]() { (void)qmini_arm::jointPositionToRotorRad(0.0, calibration); },
      "uncalibrated position command was accepted");
}

void testTrajectory() {
  constexpr double kPi = 3.14159265358979323846;
  qmini_arm::SineTrajectoryConfig trajectory;
  trajectory.amplitude_rad = 1.0;
  trajectory.center_rad = 0.0;
  trajectory.period_s = 4.0;
  trajectory.ramp_s = 1.0;

  require(near(qmini_arm::smoothStep01(-1.0), 0.0),
          "smoothstep lower clamp failed");
  require(near(qmini_arm::smoothStep01(0.5), 0.5),
          "smoothstep midpoint failed");
  require(near(qmini_arm::smoothStep01(2.0), 1.0),
          "smoothstep upper clamp failed");
  require(near(qmini_arm::sinePositionOffsetRad(trajectory, 0.0), 0.0),
          "trajectory did not start at zero");
  require(near(qmini_arm::sinePositionOffsetRad(trajectory, 1.0), 1.0),
          "trajectory peak failed");
  require(near(qmini_arm::conservativePeakSpeedRadS(trajectory),
               kPi / 2.0 + 1.5),
          "trajectory speed bound failed");
}

void testSafety() {
  qmini_arm::MotorState state;
  state.motor_id = 0;
  state.mode = 1;
  state.temperature_c = 30;
  state.position_rad = 0.0;
  state.velocity_rad_s = 0.0;
  state.torque_estimate_nm = 0.0;
  qmini_arm::validateBasicState(state);

  state.error_code = 2;
  requireThrows([&]() { qmini_arm::validateBasicState(state); },
                "motor fault was not rejected");
  state.error_code = 0;

  qmini_arm::SafetyLimits limits;
  state.velocity_rad_s = 6.33;
  requireThrows(
      [&]() {
        qmini_arm::enforceMotionSafety(state, 6.33, 0.0, limits);
      },
      "excessive output speed was not rejected");
}

void testGravityModelMatchesMujoco() {
  qmini_arm::GravityModel model;
  const qmini_arm::JointVector zero{};
  const qmini_arm::JointVector expected_zero = {
      4.5928346755190465e-06,
      0.004264811655807764,
      -0.0018557320580040717,
      3.3051717945577086e-06,
  };
  const auto actual_zero = model.compensationTorque(zero);
  for (std::size_t index = 0; index < actual_zero.size(); ++index) {
    require(near(actual_zero[index], expected_zero[index], 1e-5),
            "zero-pose gravity torque disagrees with MuJoCo");
  }

  const qmini_arm::JointVector pose = {0.2, -0.1, 0.3, 0.05};
  const qmini_arm::JointVector expected = {
      0.0008439168539749143,
      1.0646758038588573,
      -0.6884093329382603,
      0.0009824622690346868,
  };
  const auto actual = model.compensationTorque(pose);
  for (std::size_t index = 0; index < actual.size(); ++index) {
    require(near(actual[index], expected[index], 1e-5),
            "gravity torque disagrees with MuJoCo");
  }

  const qmini_arm::JointVector folded_pose = {-0.8, 0.4, -1.2, 0.9};
  const qmini_arm::JointVector expected_folded = {
      -0.0018591964794433572,
      -3.2190289941039225,
      1.762587745421884,
      -0.0017140100715784048,
  };
  const auto actual_folded = model.compensationTorque(folded_pose);
  for (std::size_t index = 0; index < actual_folded.size(); ++index) {
    require(near(actual_folded[index], expected_folded[index], 1e-5),
            "folded-pose gravity torque disagrees with MuJoCo");
  }
}

void testGuardedGravityCommandShaping() {
  const qmini_arm::JointVector joint = {6.33, -6.33, 0.0, 3.165};
  const qmini_arm::JointArray<int> directions = {1, -1, 1, 1};
  const auto rotor =
      qmini_arm::jointGravityToRotorTorque(joint, 1.0, directions, 6.33);
  require(near(rotor[0], 1.0), "100% gravity torque conversion failed");
  require(near(rotor[1], 1.0), "direction torque conversion failed");

  const qmini_arm::JointVector previous{};
  const qmini_arm::JointVector caps = {0.1, 0.1, 0.1, 0.1};
  bool saturated = false;
  const auto limited =
      qmini_arm::limitRotorTorque(rotor, previous, caps, 0.01, &saturated);
  require(saturated, "torque cap/slew did not report saturation");
  require(near(limited[0], 0.01), "torque slew limit failed");

  require(near(qmini_arm::symmetricRampEnvelope(0.0, 10.0, 2.0), 0.0),
          "ramp start failed");
  require(near(qmini_arm::symmetricRampEnvelope(1.0, 10.0, 2.0), 0.5),
          "ramp up failed");
  require(near(qmini_arm::symmetricRampEnvelope(5.0, 10.0, 2.0), 1.0),
          "ramp hold failed");
  require(near(qmini_arm::symmetricRampEnvelope(9.0, 10.0, 2.0), 0.5),
          "ramp down failed");
  require(near(qmini_arm::symmetricRampEnvelope(10.0, 10.0, 2.0), 0.0),
          "ramp end failed");
}

void testPerJointSpeedGuard() {
  const qmini_arm::JointVector soft = {0.5, 0.5, 0.5, 0.7};
  const qmini_arm::JointVector hard = {1.0, 1.0, 1.0, 1.4};
  qmini_arm::PerJointSpeedGuard guard(soft, hard, 2);
  qmini_arm::JointVector velocity{};

  // Different joints and a below-threshold sample must not share or retain a
  // consecutive violation count.
  velocity[0] = 0.6;
  guard.observeFrame(velocity);
  velocity[0] = 0.0;
  velocity[1] = 0.6;
  guard.observeFrame(velocity);
  velocity[1] = 0.0;
  velocity[0] = 0.6;
  guard.observeFrame(velocity);
  requireThrows([&]() { guard.observeFrame(velocity); },
                "consecutive soft speed trip was not enforced");

  guard.reset();
  velocity = {};
  velocity.back() = 1.5;
  requireThrows([&]() { guard.observeFrame(velocity); },
                "single-frame hard speed trip was not enforced");
  requireThrows([&]() { guard.enforceHardTrip(qmini_arm::kJointCount, 0.0); },
                "invalid joint speed index was accepted");
}

std::string zeroHomeCsv() {
  return
      "time_s,joint_1_position_rad,joint_2_position_rad,"
      "joint_3_position_rad,joint_4_position_rad,joint_1_velocity_rad_s,"
      "joint_2_velocity_rad_s,joint_3_velocity_rad_s,"
      "joint_4_velocity_rad_s\n"
      "0,0,0,0,0,0,0,0,0\n"
      "0.02,0,0,0,0,0,0,0,0\n";
}

void testHomeTrajectoryContract() {
  std::istringstream valid_stream(zeroHomeCsv());
  const qmini_arm::JointTrajectory trajectory =
      qmini_arm::parseJointTrajectoryCsv(valid_stream);
  require(trajectory.size() == 2, "home trajectory CSV row count failed");
  const qmini_arm::JointVector lower = {-1, -1, -1, -1};
  const qmini_arm::JointVector upper = {1, 1, 1, 1};
  const qmini_arm::JointVector goal{};
  qmini_arm::validateHomeTrajectory(
      trajectory, lower, upper, lower, upper, goal, false, 0.02, 0.3, 0.6,
      0.05, 120.0, 1e-4);

  const qmini_arm::JointVector hard_lower = {-2, -2, -2, -2};
  const qmini_arm::JointVector hard_upper = {2, 2, 2, 2};
  const qmini_arm::JointVector hard_goal = {0, 1.8, 0, -1.8};
  qmini_arm::JointTrajectory calibration = trajectory;
  calibration[0].position_rad = hard_goal;
  calibration[1].position_rad = hard_goal;
  qmini_arm::validateHomeTrajectory(
      calibration, lower, upper, hard_lower, hard_upper, hard_goal, true, 0.02,
      0.3, 0.6, 0.05, 120.0, 1e-4);

  qmini_arm::JointTrajectory too_fast = trajectory;
  too_fast[1].velocity_rad_s[3] = 0.31;
  requireThrows(
      [&]() {
        qmini_arm::validateHomeTrajectory(
            too_fast, lower, upper, lower, upper, goal, false, 0.02, 0.3, 0.6,
            0.05, 120.0, 1e-4);
      },
      "excessive home trajectory velocity was accepted");

  std::istringstream invalid_header("bad_header\n");
  requireThrows(
      [&]() {
        (void)qmini_arm::parseJointTrajectoryCsv(invalid_header);
      },
      "invalid home trajectory header was accepted");

  std::ostringstream legacy_csv;
  legacy_csv << "time_s";
  for (int joint = 1; joint <= 6; ++joint) {
    legacy_csv << ",joint_" << joint << "_position_rad";
  }
  for (int joint = 1; joint <= 6; ++joint) {
    legacy_csv << ",joint_" << joint << "_velocity_rad_s";
  }
  legacy_csv << '\n';
  std::istringstream legacy_stream(legacy_csv.str());
  requireThrows(
      [&]() { (void)qmini_arm::parseJointTrajectoryCsv(legacy_stream); },
      "legacy six-joint trajectory was accepted");

  const std::string header = zeroHomeCsv().substr(0, zeroHomeCsv().find('\n'));
  std::istringstream extra_values(header + "\n0,0,0,0,0,0,0,0,0,0,0,0,0\n");
  requireThrows(
      [&]() { (void)qmini_arm::parseJointTrajectoryCsv(extra_values); },
      "six-joint trajectory row under a four-joint header was accepted");

  std::istringstream trailing_value(header + "\n0,0,0,0,0,0,0,0,0,\n");
  requireThrows(
      [&]() { (void)qmini_arm::parseJointTrajectoryCsv(trailing_value); },
      "trajectory with an extra empty column was accepted");
}

}  // namespace

int main() {
  try {
    testJointConversion();
    testTrajectory();
    testSafety();
    testGravityModelMatchesMujoco();
    testGuardedGravityCommandShaping();
    testPerJointSpeedGuard();
    testHomeTrajectoryContract();
    std::cout << "All qmini_arm core tests passed.\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "TEST FAILED: " << error.what() << '\n';
    return 1;
  }
}
