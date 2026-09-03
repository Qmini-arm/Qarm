#include "qmini_arm/safety.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace qmini_arm {

namespace {

std::string prefix(const MotorState& state) {
  return "motor ID " + std::to_string(state.motor_id) + ": ";
}

}  // namespace

void validateBasicState(const MotorState& state, int expected_mode,
                        int temperature_limit_c) {
  const std::string error_prefix = prefix(state);
  if (!std::isfinite(state.position_rad) ||
      !std::isfinite(state.velocity_rad_s) ||
      !std::isfinite(state.torque_estimate_nm)) {
    throw std::runtime_error(error_prefix + "non-finite motor feedback");
  }
  if (state.mode != expected_mode) {
    throw std::runtime_error(error_prefix + "unexpected feedback mode=" +
                             std::to_string(state.mode) + ", expected " +
                             std::to_string(expected_mode));
  }
  if (state.error_code != 0) {
    throw std::runtime_error(error_prefix + "motor fault merror=" +
                             std::to_string(state.error_code));
  }
  if (state.temperature_c >= temperature_limit_c) {
    throw std::runtime_error(error_prefix + "temperature limit reached: " +
                             std::to_string(state.temperature_c) + " C");
  }
}

void enforceMotionSafety(const MotorState& state, double gear_ratio,
                         double start_rotor_position_rad,
                         const SafetyLimits& limits) {
  validateBasicState(state, limits.expected_mode, limits.temperature_limit_c);
  if (!std::isfinite(gear_ratio) || gear_ratio <= 0.0) {
    throw std::invalid_argument("gear_ratio must be positive");
  }

  const std::string error_prefix = prefix(state);
  const double output_speed = std::abs(state.velocity_rad_s / gear_ratio);
  if (output_speed > limits.output_speed_limit_rad_s) {
    throw std::runtime_error(error_prefix +
                             "output speed limit exceeded: " +
                             std::to_string(output_speed) + " rad/s");
  }
  if (std::abs(state.torque_estimate_nm) >
      limits.rotor_torque_limit_nm) {
    throw std::runtime_error(error_prefix +
                             "rotor torque estimate limit exceeded: " +
                             std::to_string(state.torque_estimate_nm) +
                             " N.m");
  }
  const double output_travel =
      std::abs(state.position_rad - start_rotor_position_rad) / gear_ratio;
  if (output_travel > limits.output_travel_limit_rad) {
    throw std::runtime_error(error_prefix +
                             "output travel limit exceeded: " +
                             std::to_string(output_travel) + " rad");
  }
}

}  // namespace qmini_arm

