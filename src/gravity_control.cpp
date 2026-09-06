#include "qmini_arm/gravity_control.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace qmini_arm {

JointVector jointGravityToRotorTorque(
    const JointVector& joint_torque_nm,
    double compensation_scale,
    const qmini_arm::JointArray<int>& directions,
    double gear_ratio) {
  if (!std::isfinite(compensation_scale) || compensation_scale < 0.0 ||
      !std::isfinite(gear_ratio) || gear_ratio <= 0.0) {
    throw std::invalid_argument("invalid gravity torque scale or gear ratio");
  }
  JointVector result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    if (!std::isfinite(joint_torque_nm[index]) ||
        (directions[index] != -1 && directions[index] != 1)) {
      throw std::invalid_argument("invalid joint torque or direction");
    }
    result[index] = compensation_scale * directions[index] *
                    joint_torque_nm[index] / gear_ratio;
  }
  return result;
}

JointVector limitRotorTorque(const JointVector& requested_nm,
                             const JointVector& previous_nm,
                             const JointVector& caps_nm,
                             double slew_nm_per_cycle,
                             bool* saturated) {
  if (!std::isfinite(slew_nm_per_cycle) || slew_nm_per_cycle <= 0.0) {
    throw std::invalid_argument("torque slew limit must be positive");
  }
  bool any_saturated = false;
  JointVector result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    if (!std::isfinite(requested_nm[index]) ||
        !std::isfinite(previous_nm[index]) || !std::isfinite(caps_nm[index]) ||
        caps_nm[index] <= 0.0) {
      throw std::invalid_argument("invalid torque limit input");
    }
    const double capped = std::max(
        -caps_nm[index], std::min(caps_nm[index], requested_nm[index]));
    const double slewed = std::max(
        previous_nm[index] - slew_nm_per_cycle,
        std::min(previous_nm[index] + slew_nm_per_cycle, capped));
    any_saturated = any_saturated ||
                    std::abs(slewed - requested_nm[index]) > 1e-12;
    result[index] = slewed;
  }
  if (saturated != nullptr) *saturated = any_saturated;
  return result;
}

double symmetricRampEnvelope(double elapsed_s,
                             double duration_s,
                             double ramp_s) {
  if (!std::isfinite(elapsed_s) || !std::isfinite(duration_s) ||
      !std::isfinite(ramp_s) || duration_s <= 0.0 || ramp_s <= 0.0 ||
      duration_s < 2.0 * ramp_s) {
    throw std::invalid_argument("invalid symmetric ramp timing");
  }
  if (elapsed_s <= 0.0 || elapsed_s >= duration_s) return 0.0;
  return std::max(
      0.0,
      std::min(1.0, std::min(elapsed_s / ramp_s,
                             (duration_s - elapsed_s) / ramp_s)));
}

PerJointSpeedGuard::PerJointSpeedGuard(
    const JointVector& soft_trip_rad_s,
    const JointVector& hard_trip_rad_s,
    std::size_t consecutive_cycles)
    : soft_trip_rad_s_(soft_trip_rad_s),
      hard_trip_rad_s_(hard_trip_rad_s),
      consecutive_cycles_(consecutive_cycles) {
  if (consecutive_cycles_ == 0) {
    throw std::invalid_argument(
        "speed trip consecutive cycle count must be positive");
  }
  for (std::size_t index = 0; index < soft_trip_rad_s_.size(); ++index) {
    if (!std::isfinite(soft_trip_rad_s_[index]) ||
        !std::isfinite(hard_trip_rad_s_[index]) ||
        soft_trip_rad_s_[index] <= 0.0 ||
        hard_trip_rad_s_[index] <= soft_trip_rad_s_[index]) {
      throw std::invalid_argument("invalid per-joint speed trip thresholds");
    }
  }
}

void PerJointSpeedGuard::reset() { consecutive_counts_.fill(0); }

void PerJointSpeedGuard::enforceHardTrip(
    std::size_t joint_index, double velocity_rad_s) const {
  if (joint_index >= hard_trip_rad_s_.size() ||
      !std::isfinite(velocity_rad_s)) {
    throw std::invalid_argument("invalid joint speed sample");
  }
  if (std::abs(velocity_rad_s) > hard_trip_rad_s_[joint_index]) {
    std::ostringstream message;
    message << "joint_" << joint_index + 1 << " speed " << std::fixed
            << std::setprecision(3) << velocity_rad_s
            << " rad/s exceeds hard trip "
            << hard_trip_rad_s_[joint_index] << " rad/s";
    throw std::runtime_error(message.str());
  }
}

void PerJointSpeedGuard::observeFrame(const JointVector& velocity_rad_s) {
  for (std::size_t index = 0; index < velocity_rad_s.size(); ++index) {
    enforceHardTrip(index, velocity_rad_s[index]);
    if (std::abs(velocity_rad_s[index]) > soft_trip_rad_s_[index]) {
      ++consecutive_counts_[index];
    } else {
      consecutive_counts_[index] = 0;
    }
    if (consecutive_counts_[index] >= consecutive_cycles_) {
      std::ostringstream message;
      message << "joint_" << index + 1 << " speed " << std::fixed
              << std::setprecision(3) << velocity_rad_s[index]
              << " rad/s exceeds soft trip " << soft_trip_rad_s_[index]
              << " rad/s for " << consecutive_cycles_
              << " consecutive cycles";
      throw std::runtime_error(message.str());
    }
  }
}

}  // namespace qmini_arm
