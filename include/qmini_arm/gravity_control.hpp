#ifndef QMINI_ARM_GRAVITY_CONTROL_HPP_
#define QMINI_ARM_GRAVITY_CONTROL_HPP_

#include <array>
#include <cstddef>

#include "qmini_arm/gravity_model.hpp"

namespace qmini_arm {

JointVector jointGravityToRotorTorque(
    const JointVector& joint_torque_nm,
    double compensation_scale,
    const std::array<int, 6>& directions,
    double gear_ratio);

JointVector limitRotorTorque(const JointVector& requested_nm,
                             const JointVector& previous_nm,
                             const JointVector& caps_nm,
                             double slew_nm_per_cycle,
                             bool* saturated);

// Linear 0->1 ramp at startup and 1->0 ramp before a normal bounded exit.
double symmetricRampEnvelope(double elapsed_s,
                             double duration_s,
                             double ramp_s);

// Debounces ordinary hand-guided motion limits while retaining a separate
// single-sample hard trip for genuinely excessive speed.
class PerJointSpeedGuard {
 public:
  PerJointSpeedGuard(const JointVector& soft_trip_rad_s,
                     const JointVector& hard_trip_rad_s,
                     std::size_t consecutive_cycles);

  void reset();
  void observeFrame(const JointVector& velocity_rad_s);
  void enforceHardTrip(std::size_t joint_index,
                       double velocity_rad_s) const;

 private:
  JointVector soft_trip_rad_s_{};
  JointVector hard_trip_rad_s_{};
  std::array<std::size_t, 6> consecutive_counts_{};
  std::size_t consecutive_cycles_ = 0;
};

}  // namespace qmini_arm

#endif  // QMINI_ARM_GRAVITY_CONTROL_HPP_
