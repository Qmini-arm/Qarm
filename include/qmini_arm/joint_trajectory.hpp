#ifndef QMINI_ARM_JOINT_TRAJECTORY_HPP_
#define QMINI_ARM_JOINT_TRAJECTORY_HPP_

#include <array>
#include <istream>
#include <string>
#include <vector>

#include "qmini_arm/gravity_model.hpp"

namespace qmini_arm {

struct JointTrajectorySample {
  double time_s = 0.0;
  JointVector position_rad{};
  JointVector velocity_rad_s{};
};

using JointTrajectory = std::vector<JointTrajectorySample>;

JointTrajectory parseJointTrajectoryCsv(std::istream& stream);
JointTrajectory loadJointTrajectoryCsv(const std::string& path);

// Validate the numeric contract that the hardware runner can independently
// check. Self-collision remains the responsibility of the offline planner and
// MuJoCo validation stage that produced the CSV.
void validateHomeTrajectory(const JointTrajectory& trajectory,
                            const JointVector& soft_lower_rad,
                            const JointVector& soft_upper_rad,
                            double limit_margin_rad,
                            double maximum_velocity_rad_s,
                            double maximum_acceleration_rad_s2,
                            double maximum_sample_period_s,
                            double maximum_duration_s,
                            double final_tolerance_rad);

}  // namespace qmini_arm

#endif  // QMINI_ARM_JOINT_TRAJECTORY_HPP_
