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
      0.000379408546167553,
      0.4622329897782038,
      -0.45724924222659047,
      0.4527995154873221,
      -0.008266270739277437,
      0.0028573351199981476,
  };
  const auto actual_zero = model.compensationTorque(zero);
  for (std::size_t index = 0; index < actual_zero.size(); ++index) {
    require(near(actual_zero[index], expected_zero[index], 1e-5),
            "zero-pose gravity torque disagrees with MuJoCo");
  }

  const qmini_arm::JointVector pose = {0.2, -0.1, 0.3, 0.05, -0.2, 0.1};
  const qmini_arm::JointVector expected = {
      0.002601902121703785,
      3.1270456019390407,
      -2.415411918094685,
      0.42586188478903986,
      -0.007495958065603048,
      0.0027133658205243175,
  };
  const auto actual = model.compensationTorque(pose);
  for (std::size_t index = 0; index < actual.size(); ++index) {
    require(near(actual[index], expected[index], 1e-5),
            "gravity torque disagrees with MuJoCo");
  }
}

void testGuardedGravityCommandShaping() {
  const qmini_arm::JointVector joint = {6.33, -6.33, 0.0, 3.165, 1.0, -1.0};
  const std::array<int, 6> directions = {1, -1, 1, 1, -1, -1};
  const auto rotor =
      qmini_arm::jointGravityToRotorTorque(joint, 1.0, directions, 6.33);
  require(near(rotor[0], 1.0), "100% gravity torque conversion failed");
  require(near(rotor[1], 1.0), "direction torque conversion failed");

  const qmini_arm::JointVector previous{};
  const qmini_arm::JointVector caps = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
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
  const qmini_arm::JointVector soft = {0.5, 0.5, 0.5, 0.7, 1.0, 1.5};
  const qmini_arm::JointVector hard = {1.0, 1.0, 1.0, 1.4, 2.0, 2.0};
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
  velocity[5] = 2.1;
  requireThrows([&]() { guard.observeFrame(velocity); },
                "single-frame hard speed trip was not enforced");
  requireThrows([&]() { guard.enforceHardTrip(6, 0.0); },
                "invalid joint speed index was accepted");
}

std::string zeroHomeCsv() {
  return
      "time_s,joint_1_position_rad,joint_2_position_rad,"
      "joint_3_position_rad,joint_4_position_rad,joint_5_position_rad,"
      "joint_6_position_rad,joint_1_velocity_rad_s,"
      "joint_2_velocity_rad_s,joint_3_velocity_rad_s,"
      "joint_4_velocity_rad_s,joint_5_velocity_rad_s,"
      "joint_6_velocity_rad_s\n"
      "0,0,0,0,0,0,0,0,0,0,0,0,0\n"
      "0.02,0,0,0,0,0,0,0,0,0,0,0,0\n";
}

void testHomeTrajectoryContract() {
  std::istringstream valid_stream(zeroHomeCsv());
  const qmini_arm::JointTrajectory trajectory =
      qmini_arm::parseJointTrajectoryCsv(valid_stream);
  require(trajectory.size() == 2, "home trajectory CSV row count failed");
  const qmini_arm::JointVector lower = {-1, -1, -1, -1, -1, -1};
  const qmini_arm::JointVector upper = {1, 1, 1, 1, 1, 1};
  qmini_arm::validateHomeTrajectory(trajectory, lower, upper, 0.02, 0.3,
                                    0.6, 0.05, 120.0, 1e-4);

  qmini_arm::JointTrajectory too_fast = trajectory;
  too_fast[1].velocity_rad_s[3] = 0.31;
  requireThrows(
      [&]() {
        qmini_arm::validateHomeTrajectory(too_fast, lower, upper, 0.02,
                                          0.3, 0.6, 0.05, 120.0, 1e-4);
      },
      "excessive home trajectory velocity was accepted");

  std::istringstream invalid_header("bad_header\n");
  requireThrows(
      [&]() {
        (void)qmini_arm::parseJointTrajectoryCsv(invalid_header);
      },
      "invalid home trajectory header was accepted");
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
