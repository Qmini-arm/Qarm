#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include "qmini_arm/joint_conversion.hpp"
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

}  // namespace

int main() {
  try {
    testJointConversion();
    testTrajectory();
    testSafety();
    std::cout << "All qmini_arm core tests passed.\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "TEST FAILED: " << error.what() << '\n';
    return 1;
  }
}

