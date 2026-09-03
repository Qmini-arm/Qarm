#include "qmini_arm/joint_conversion.hpp"

#include <cmath>
#include <stdexcept>

namespace qmini_arm {

void validateCalibration(const JointCalibration& calibration) {
  if (calibration.motor_id < 0 || calibration.motor_id > 14) {
    throw std::invalid_argument("calibration motor_id must be in [0, 14]");
  }
  if (calibration.direction != -1 && calibration.direction != 1) {
    throw std::invalid_argument("calibration direction must be +1 or -1");
  }
  if (!std::isfinite(calibration.gear_ratio) ||
      calibration.gear_ratio <= 0.0) {
    throw std::invalid_argument("calibration gear_ratio must be positive");
  }
  if (!std::isfinite(calibration.rotor_zero_rad) ||
      !std::isfinite(calibration.joint_zero_rad)) {
    throw std::invalid_argument("calibration zero values must be finite");
  }
}

double rawOutputPositionRad(const MotorState& motor_state,
                            double gear_ratio) {
  if (!std::isfinite(gear_ratio) || gear_ratio <= 0.0) {
    throw std::invalid_argument("gear_ratio must be positive");
  }
  return motor_state.position_rad / gear_ratio;
}

JointState toJointState(const MotorState& motor_state,
                        const JointCalibration& calibration) {
  validateCalibration(calibration);
  if (motor_state.motor_id != calibration.motor_id) {
    throw std::invalid_argument("motor state ID does not match calibration ID");
  }

  JointState result;
  result.motor_id = motor_state.motor_id;
  result.position_calibrated = calibration.position_calibrated;
  if (calibration.position_calibrated) {
    result.position_rad =
        calibration.joint_zero_rad +
        calibration.direction *
            (motor_state.position_rad - calibration.rotor_zero_rad) /
            calibration.gear_ratio;
  }
  result.velocity_rad_s = calibration.direction * motor_state.velocity_rad_s /
                          calibration.gear_ratio;
  result.torque_estimate_nm =
      calibration.direction * motor_state.torque_estimate_nm *
      calibration.gear_ratio;
  return result;
}

double jointPositionToRotorRad(double joint_position_rad,
                               const JointCalibration& calibration) {
  validateCalibration(calibration);
  if (!calibration.position_calibrated) {
    throw std::invalid_argument(
        "joint position command requires a calibrated rotor zero");
  }
  return calibration.rotor_zero_rad +
         calibration.direction * calibration.gear_ratio *
             (joint_position_rad - calibration.joint_zero_rad);
}

double jointVelocityToRotorRadS(double joint_velocity_rad_s,
                                const JointCalibration& calibration) {
  validateCalibration(calibration);
  return calibration.direction * calibration.gear_ratio *
         joint_velocity_rad_s;
}

double jointTorqueToRotorNm(double joint_torque_nm,
                            const JointCalibration& calibration) {
  validateCalibration(calibration);
  return calibration.direction * joint_torque_nm / calibration.gear_ratio;
}

double jointKpToRotor(double joint_kp,
                      const JointCalibration& calibration) {
  validateCalibration(calibration);
  return joint_kp /
         (calibration.gear_ratio * calibration.gear_ratio);
}

double jointKdToRotor(double joint_kd,
                      const JointCalibration& calibration) {
  validateCalibration(calibration);
  return joint_kd /
         (calibration.gear_ratio * calibration.gear_ratio);
}

}  // namespace qmini_arm

