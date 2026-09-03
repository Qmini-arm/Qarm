#ifndef QMINI_ARM_TYPES_HPP_
#define QMINI_ARM_TYPES_HPP_

#include <limits>

namespace qmini_arm {

// Values accepted by the Unitree FOC force/position hybrid controller.
// They are all rotor-side quantities; no gear-ratio conversion is implicit.
struct MotorCommand {
  double torque_ff_nm = 0.0;
  double velocity_rad_s = 0.0;
  double position_rad = 0.0;
  double kp = 0.0;
  double kd = 0.0;
};

// Feedback decoded by unitree_actuator_sdk. Position, velocity and torque are
// rotor-side values. The M8010 response does not directly contain a calibrated
// robot-joint angle.
struct MotorState {
  int motor_id = -1;
  double position_rad = 0.0;
  double velocity_rad_s = 0.0;
  double torque_estimate_nm = 0.0;
  int temperature_c = 0;
  int error_code = 0;
  int mode = 0;
};

// Maps a motor encoder coordinate to a URDF/mechanical joint coordinate:
//   q_joint = joint_zero + direction * (q_rotor - rotor_zero) / gear_ratio
// A calibrated absolute joint position requires rotor_zero_rad to be obtained
// from a repeatable homing procedure or an output-side absolute sensor.
struct JointCalibration {
  int motor_id = -1;
  int direction = 1;
  double gear_ratio = 6.33;
  double rotor_zero_rad = 0.0;
  double joint_zero_rad = 0.0;
  bool position_calibrated = false;
};

struct JointState {
  int motor_id = -1;
  bool position_calibrated = false;
  double position_rad = std::numeric_limits<double>::quiet_NaN();
  double velocity_rad_s = 0.0;
  // Ideal gear-ratio conversion of the SDK rotor torque estimate. It does not
  // account for gearbox efficiency, friction, structural load or calibration.
  double torque_estimate_nm = 0.0;
};

}  // namespace qmini_arm

#endif  // QMINI_ARM_TYPES_HPP_

