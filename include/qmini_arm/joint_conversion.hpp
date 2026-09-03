#ifndef QMINI_ARM_JOINT_CONVERSION_HPP_
#define QMINI_ARM_JOINT_CONVERSION_HPP_

#include "qmini_arm/types.hpp"

namespace qmini_arm {

void validateCalibration(const JointCalibration& calibration);

// Raw reducer-output position. This is useful for diagnostics but is not an
// absolute robot-joint angle until direction and mechanical zero are calibrated.
double rawOutputPositionRad(const MotorState& motor_state,
                            double gear_ratio);

JointState toJointState(const MotorState& motor_state,
                        const JointCalibration& calibration);

double jointPositionToRotorRad(double joint_position_rad,
                               const JointCalibration& calibration);

double jointVelocityToRotorRadS(double joint_velocity_rad_s,
                                const JointCalibration& calibration);

// Ideal conversion used for command planning. Application-level controllers
// should still impose motor-side and joint-side torque limits.
double jointTorqueToRotorNm(double joint_torque_nm,
                            const JointCalibration& calibration);

double jointKpToRotor(double joint_kp,
                      const JointCalibration& calibration);

double jointKdToRotor(double joint_kd,
                      const JointCalibration& calibration);

}  // namespace qmini_arm

#endif  // QMINI_ARM_JOINT_CONVERSION_HPP_

