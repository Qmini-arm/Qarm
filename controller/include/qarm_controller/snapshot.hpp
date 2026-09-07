#ifndef QARM_CONTROLLER_SNAPSHOT_HPP_
#define QARM_CONTROLLER_SNAPSHOT_HPP_
#include "qarm_controller/command_protocol.hpp"
#include "qarm_controller/control_state.hpp"

namespace qarm_controller {
struct ControllerSnapshot {
  int schema_version = 1;
  std::uint64_t sequence = 0;
  std::uint64_t monotonic_ns = 0;
  ControllerState state = ControllerState::Disconnected;
  std::string model_hash;
  std::string calibration_id;
  std::string board_boot_id;
  FeedbackFrame feedback;
  // NaN internally while uncalibrated; serializers must emit JSON null.
  JointArray q_joint{};
  JointArray dq_joint{};
  JointArray q_des{};
  JointArray dq_des{};
  JointArray tau_ff{};
  JointArray tracking_error{};
  double gravity_scale = 0.0;
  std::string active_plan_id;
  std::string fault;
  std::size_t zero_capture_samples = 0;
  JointArray capture_rotor_mean{};
  bool connected = false;
  bool lease_active = false;
  bool position_calibrated = false;
};
// Consumed only by a local bus adapter; never a frontend command.
struct ControlOutput {
  bool brake = true;
  std::array<int, kJointCount> motor_ids{{0, 1, 2, 3}};
  std::array<qmini_arm::MotorCommand, kJointCount> motors{};
};
}  // namespace qarm_controller
#endif
