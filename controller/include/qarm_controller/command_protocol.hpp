#ifndef QARM_CONTROLLER_COMMAND_PROTOCOL_HPP_
#define QARM_CONTROLLER_COMMAND_PROTOCOL_HPP_

#include <array>
#include <cstdint>
#include <string>
#include "qmini_arm/joint_trajectory.hpp"
#include "qmini_arm/types.hpp"

namespace qarm_controller {
constexpr std::size_t kJointCount = qmini_arm::kJointCount;
using JointArray = qmini_arm::JointVector;
using JointTrajectory = qmini_arm::JointTrajectory;
enum class CommandKind {
  ZeroCapture, ZeroCommit, GravitySet, PlanValidate, PlanExecute,
  Stop, Estop, LeaseHeartbeat, ResetFault,
};
inline const char* commandKindName(CommandKind kind) {
  switch (kind) {
    case CommandKind::ZeroCapture: return "zero.capture";
    case CommandKind::ZeroCommit: return "zero.commit";
    case CommandKind::GravitySet: return "gravity.set";
    case CommandKind::PlanValidate: return "plan.validate";
    case CommandKind::PlanExecute: return "plan.execute";
    case CommandKind::Stop: return "stop";
    case CommandKind::Estop: return "estop";
    case CommandKind::LeaseHeartbeat: return "lease.heartbeat";
    case CommandKind::ResetFault: return "fault.reset";
  }
  return "unknown";
}
struct FeedbackFrame {
  std::uint64_t sequence = 0;
  std::uint64_t monotonic_ns = 0;
  std::array<qmini_arm::MotorState, kJointCount> motors{};
  std::array<bool, kJointCount> valid{{false, false, false, false}};
};
struct CalibrationMetadata {
  std::string calibration_id;
  std::string model_hash;
  std::string board_boot_id;
  std::array<qmini_arm::JointCalibration, kJointCount> joints{};
  bool directions_confirmed = false;
  bool reference_pose_confirmed = false;
};
// Copied by value on validation. Collision checking belongs to the trusted
// planner; the core verifies its attestation, not geometry or peer identity.
struct Plan {
  std::string plan_id;
  std::string model_hash;
  std::string calibration_id;
  std::string board_boot_id;
  bool collision_checked = false;
  JointTrajectory trajectory;
};
struct Command {
  CommandKind kind = CommandKind::LeaseHeartbeat;
  std::string request_id;
  std::string lease_id;
  std::string model_hash;
  std::string calibration_id;
  std::string board_boot_id;
  std::string plan_id;
  double gravity_scale = 0.0;
  bool external_support_confirmed = false;
  CalibrationMetadata calibration;
  Plan plan;
};
}  // namespace qarm_controller
#endif
