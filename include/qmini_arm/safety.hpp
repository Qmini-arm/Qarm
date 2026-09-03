#ifndef QMINI_ARM_SAFETY_HPP_
#define QMINI_ARM_SAFETY_HPP_

#include "qmini_arm/types.hpp"

namespace qmini_arm {

struct SafetyLimits {
  double output_speed_limit_rad_s = 0.5;
  double rotor_torque_limit_nm = 1.0;
  double output_travel_limit_rad = 15.0 * 3.14159265358979323846 / 180.0;
  int temperature_limit_c = 60;
  int expected_mode = 1;
};

// Validates CRC-decoded state semantics that are useful for both monitoring and
// active control. Throws std::runtime_error with the motor ID on failure.
void validateBasicState(const MotorState& state, int expected_mode = 1,
                        int temperature_limit_c = 60);

// Adds motion-specific speed, torque and relative-travel guards.
void enforceMotionSafety(const MotorState& state, double gear_ratio,
                         double start_rotor_position_rad,
                         const SafetyLimits& limits);

}  // namespace qmini_arm

#endif  // QMINI_ARM_SAFETY_HPP_

