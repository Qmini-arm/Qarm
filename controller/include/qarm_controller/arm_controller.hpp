#ifndef QARM_CONTROLLER_ARM_CONTROLLER_HPP_
#define QARM_CONTROLLER_ARM_CONTROLLER_HPP_
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include "qarm_controller/snapshot.hpp"
#include "qmini_arm/gravity_model.hpp"

namespace qarm_controller {
struct ControllerConfig {
  std::string model_hash;
  std::string board_boot_id;
  std::array<int, kJointCount> motor_ids{{0, 1, 2, 3}};
  JointArray gear_ratios{{6.33, 6.33, 6.33, 6.33}};
  JointArray soft_lower{{-1.0, -1.0, -1.0, -1.0}};
  JointArray soft_upper{{1.0, 1.0, 1.0, 1.0}};
  JointArray hard_lower{{-2.0, -2.0, -2.0, -2.0}};
  JointArray hard_upper{{2.0, 2.0, 2.0, 2.0}};
  JointArray rotor_torque_caps{{0.2, 0.2, 0.2, 0.2}};
  JointArray position_kp{{5.0, 5.0, 5.0, 5.0}};
  JointArray velocity_kd{{0.1, 0.1, 0.1, 0.1}};
  double maximum_velocity = 0.5;
  double maximum_acceleration = 1.0;
  double maximum_sample_period = 0.05;
  double maximum_duration = 120.0;
  double start_tolerance = 0.02;
  double tracking_tolerance = 0.15;
  double limit_margin = 0.01;
  double capture_span_rotor = 0.02;
  double rotor_torque_slew_per_s = 0.5;
  double gravity_ramp_per_s = 0.5;
  double lease_timeout_s = 2.0;
  double feedback_timeout_s = 0.1;
  double maximum_tick_s = 0.05;
  int temperature_limit_c = 60;
};

// Thread-safe in-process core; no socket, serial port or hardware thread.
class ArmController {
 public:
  using Clock = std::function<std::uint64_t()>;
  explicit ArmController(ControllerConfig config, Clock clock = Clock());
  void connect();
  void disconnect();
  // Bus/transport supervisor reports failures here; this never clears ESTOP.
  void reportFault(const std::string& reason);
  bool handleCommand(const Command& command, std::string* error = nullptr);
  bool updateFeedback(const FeedbackFrame& feedback, std::string* error = nullptr);
  bool tick(double dt_s, std::string* error = nullptr);
  ControllerSnapshot snapshot() const;
  ControlOutput output() const;
  ControllerConfig config() const;

 private:
  bool reject(std::string* error, const std::string& reason) const;
  bool requireLease(const Command& command, std::string* error);
  bool requireIdentity(const Command& command, std::string* error) const;
  bool freshFeedback() const;
  bool withinSoftLimits() const;
  bool feedbackHealthy(std::string* error) const;
  bool validateCalibration(const CalibrationMetadata&, std::string*) const;
  bool validatePlan(const Plan& plan, std::string* error) const;
  void updateJointState();
  void refresh(std::uint64_t now);
  void clearPlans();
  void stopOutput();
  void fault(const std::string& reason);
  void expireLease();
  void resetCalibration();
  mutable std::mutex mutex_;
  ControllerConfig config_;
  Clock clock_;
  ControllerSnapshot snapshot_;
  ControlOutput output_;
  CalibrationMetadata calibration_;
  std::string lease_id_;
  std::uint64_t lease_deadline_ns_ = 0;
  std::uint64_t feedback_received_ns_ = 0;
  std::uint64_t last_tick_ns_ = 0;
  std::uint64_t execution_start_ns_ = 0;
  bool have_feedback_ = false;
  double gravity_target_ = 0.0;
  JointArray capture_min_{};
  JointArray capture_max_{};
  std::map<std::string, Plan> plans_;
  qmini_arm::GravityModel gravity_model_;
};

// Ideal tracking fixture, not a physics simulator. Calibration starts invalid.
class FakeController {
 public:
  explicit FakeController(ControllerConfig config);
  ArmController& controller() { return controller_; }
  void setRotorPosition(const JointArray& position);
  bool advance(double dt_s, std::string* error = nullptr);
 private:
  std::uint64_t now_ns_ = 1000000000ULL;
  std::uint64_t sequence_ = 0;
  JointArray rotor_position_{};
  ArmController controller_;
};
}  // namespace qarm_controller
#endif
