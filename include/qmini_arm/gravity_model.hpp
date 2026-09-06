#ifndef QMINI_ARM_GRAVITY_MODEL_HPP_
#define QMINI_ARM_GRAVITY_MODEL_HPP_

#include "qmini_arm/types.hpp"

namespace qmini_arm {

// Gravity-only inverse dynamics generated from qmini_arm.urdf.xacro.
// The returned vector is the joint torque that holds a static pose against
// gravity (the same convention as MuJoCo qfrc_bias with qvel=0).
class GravityModel {
 public:
  JointVector compensationTorque(const JointVector& joint_position_rad) const;
};

}  // namespace qmini_arm

#endif  // QMINI_ARM_GRAVITY_MODEL_HPP_
