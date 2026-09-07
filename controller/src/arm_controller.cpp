#include "qarm_controller/arm_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>
#include "qmini_arm/gravity_control.hpp"
#include "qmini_arm/joint_conversion.hpp"
#include "qmini_arm/safety.hpp"

namespace qarm_controller {
namespace {
std::uint64_t steadyNs() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}
std::uint64_t ns(double seconds) {
  return static_cast<std::uint64_t>(seconds * 1e9);
}
bool powered(ControllerState state) {
  return state == ControllerState::GravityHold || state == ControllerState::Executing;
}
bool positive(double value) { return std::isfinite(value) && value > 0.0; }
double clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}
// Cubic Hermite interpolation is shared by validation and execution. Every
// position/velocity extremum and both acceleration endpoints are checked.
struct Cubic {
  double a, b, c, d, dt;
  double q(double u) const { return ((a * u + b) * u + c) * u + d; }
  double v(double u) const { return (3 * a * u * u + 2 * b * u + c) / dt; }
  double acc(double u) const { return (6 * a * u + 2 * b) / (dt * dt); }
};
Cubic segment(const qmini_arm::JointTrajectorySample& x,
              const qmini_arm::JointTrajectorySample& y, std::size_t j) {
  const double dt = y.time_s - x.time_s;
  const double delta = y.position_rad[j] - x.position_rad[j];
  const double c = x.velocity_rad_s[j] * dt;
  const double end = y.velocity_rad_s[j] * dt;
  return {c + end - 2 * delta, 3 * delta - 2 * c - end,
          c, x.position_rad[j], dt};
}
void checkSegment(const Cubic& p, double lower, double upper,
                  double max_v, double max_a) {
  std::vector<double> points{0.0, 1.0};
  if (std::abs(p.a) > 1e-15) {
    points.push_back(-p.b / (3 * p.a));
    const double discriminant = 4 * p.b * p.b - 12 * p.a * p.c;
    if (discriminant >= 0) {
      points.push_back((-2 * p.b + std::sqrt(discriminant)) / (6 * p.a));
      points.push_back((-2 * p.b - std::sqrt(discriminant)) / (6 * p.a));
    }
  } else if (std::abs(p.b) > 1e-15) {
    points.push_back(-p.c / (2 * p.b));
  }
  for (double u : points) {
    if (u < 0 || u > 1) continue;
    if (!std::isfinite(p.q(u)) || !std::isfinite(p.v(u)) || !std::isfinite(p.acc(u)) ||
        p.q(u) <= lower || p.q(u) >= upper ||
        std::abs(p.v(u)) > max_v + 1e-8 ||
        std::abs(p.acc(u)) > max_a + 1e-8) {
      throw std::runtime_error("trajectory interpolation violates position, speed or acceleration limits");
    }
  }
}
}  // namespace

ArmController::ArmController(ControllerConfig config, Clock clock)
    : config_(std::move(config)), clock_(clock ? std::move(clock) : Clock(steadyNs)) {
  if (config_.model_hash.empty() || config_.board_boot_id.empty())
    throw std::invalid_argument("model_hash and board_boot_id are required");
  const double positive_values[] = {
      config_.maximum_velocity, config_.maximum_acceleration,
      config_.maximum_sample_period, config_.maximum_duration,
      config_.start_tolerance, config_.tracking_tolerance,
      config_.capture_span_rotor, config_.rotor_torque_slew_per_s,
      config_.gravity_ramp_per_s, config_.lease_timeout_s,
      config_.feedback_timeout_s, config_.maximum_tick_s};
  for (double value : positive_values)
    if (!positive(value)) throw std::invalid_argument("controller limits must be finite and positive");
  if (!std::isfinite(config_.limit_margin) || config_.limit_margin < 0 ||
      config_.temperature_limit_c <= 0)
    throw std::invalid_argument("invalid margin or temperature limit");
  std::set<int> ids;
  for (std::size_t j = 0; j < kJointCount; ++j) {
    if (config_.motor_ids[j] < 0 || config_.motor_ids[j] > 14 || !ids.insert(config_.motor_ids[j]).second ||
        !positive(config_.gear_ratios[j]) || !positive(config_.rotor_torque_caps[j]) ||
        !positive(config_.position_kp[j]) || !positive(config_.velocity_kd[j]) ||
        !std::isfinite(config_.hard_lower[j]) || !std::isfinite(config_.hard_upper[j]) ||
        !std::isfinite(config_.soft_lower[j]) || !std::isfinite(config_.soft_upper[j]) ||
        config_.hard_lower[j] > config_.soft_lower[j] ||
        config_.hard_upper[j] < config_.soft_upper[j] ||
        config_.soft_lower[j] + 2 * config_.limit_margin >= config_.soft_upper[j])
      throw std::invalid_argument("invalid joint mapping, limits or gains");
  }
  snapshot_.model_hash = config_.model_hash;
  snapshot_.board_boot_id = config_.board_boot_id;
  output_.motor_ids = config_.motor_ids;
  resetCalibration();
  refresh(clock_());
}

void ArmController::refresh(std::uint64_t now) {
  snapshot_.monotonic_ns = now;
  ++snapshot_.sequence;
}
bool ArmController::reject(std::string* error, const std::string& reason) const {
  if (error) *error = reason;
  return false;
}
void ArmController::clearPlans() { plans_.clear(); snapshot_.active_plan_id.clear(); }
void ArmController::stopOutput() {
  output_.brake = true;
  output_.motors = {};
  snapshot_.tau_ff = {};
  snapshot_.dq_des = {};
  snapshot_.gravity_scale = 0;
  gravity_target_ = 0;
  snapshot_.active_plan_id.clear();
}
void ArmController::resetCalibration() {
  calibration_ = {};
  snapshot_.calibration_id.clear();
  snapshot_.position_calibrated = false;
  snapshot_.q_joint.fill(std::numeric_limits<double>::quiet_NaN());
  snapshot_.q_des = snapshot_.q_joint;
  snapshot_.zero_capture_samples = 0;
  clearPlans();
}
void ArmController::fault(const std::string& reason) {
  stopOutput();
  clearPlans();
  snapshot_.fault = reason;
  if (snapshot_.state != ControllerState::Estop) snapshot_.state = ControllerState::Fault;
}
void ArmController::connect() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (snapshot_.connected) return;
  snapshot_.connected = true;
  have_feedback_ = false;
  last_tick_ns_ = clock_();
  if (snapshot_.state == ControllerState::Disconnected) snapshot_.state = ControllerState::ReadOnly;
  refresh(clock_());
}
void ArmController::disconnect() {
  std::lock_guard<std::mutex> lock(mutex_);
  stopOutput();
  resetCalibration();
  snapshot_.connected = false;
  snapshot_.lease_active = false;
  lease_id_.clear();
  have_feedback_ = false;
  if (snapshot_.state != ControllerState::Estop && snapshot_.state != ControllerState::Fault)
    snapshot_.state = ControllerState::Disconnected;
  refresh(clock_());
}
void ArmController::reportFault(const std::string& reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  fault(reason);
  refresh(clock_());
}
void ArmController::expireLease() {
  if (!lease_id_.empty() && clock_() >= lease_deadline_ns_) {
    lease_id_.clear();
    snapshot_.lease_active = false;
    if (powered(snapshot_.state) || snapshot_.state == ControllerState::ZeroCapture)
      fault("control lease expired");
    clearPlans();
  }
}
bool ArmController::requireLease(const Command& command, std::string* error) {
  expireLease();
  if (!snapshot_.lease_active || command.lease_id.empty() || command.lease_id != lease_id_)
    return reject(error, "command requires the current unexpired control lease");
  return true;
}
bool ArmController::requireIdentity(const Command& command, std::string* error) const {
  if (command.model_hash != config_.model_hash || command.board_boot_id != config_.board_boot_id ||
      !snapshot_.position_calibrated || command.calibration_id != snapshot_.calibration_id)
    return reject(error, "command identity does not match the current model, boot and calibration");
  return true;
}
bool ArmController::freshFeedback() const {
  const auto now = clock_();
  return have_feedback_ && now >= feedback_received_ns_ &&
         now - feedback_received_ns_ < ns(config_.feedback_timeout_s);
}
bool ArmController::withinSoftLimits() const {
  if (!snapshot_.position_calibrated) return false;
  for (std::size_t j = 0; j < kJointCount; ++j)
    if (!std::isfinite(snapshot_.q_joint[j]) ||
        snapshot_.q_joint[j] <= config_.soft_lower[j] + config_.limit_margin ||
        snapshot_.q_joint[j] >= config_.soft_upper[j] - config_.limit_margin) return false;
  return true;
}
bool ArmController::feedbackHealthy(std::string* error) const {
  if (!freshFeedback()) return reject(error, "fresh feedback is required");
  try {
    for (std::size_t j = 0; j < kJointCount; ++j) {
      const auto& motor = snapshot_.feedback.motors[j];
      if (!snapshot_.feedback.valid[j] || motor.motor_id != config_.motor_ids[j])
        return reject(error, "missing or mismatched motor feedback");
      qmini_arm::validateBasicState(motor, output_.brake ? 0 : 1, config_.temperature_limit_c);
      if (std::abs(motor.velocity_rad_s / config_.gear_ratios[j]) > config_.maximum_velocity)
        return reject(error, "motor feedback exceeds the speed limit");
      if (!output_.brake && std::abs(motor.torque_estimate_nm) > config_.rotor_torque_caps[j] + 1e-6)
        return reject(error, "motor torque feedback exceeds its configured cap");
    }
  } catch (const std::exception& e) { return reject(error, e.what()); }
  return true;
}
void ArmController::updateJointState() {
  if (!snapshot_.position_calibrated) return;
  for (std::size_t j = 0; j < kJointCount; ++j) {
    const auto joint = qmini_arm::toJointState(snapshot_.feedback.motors[j], calibration_.joints[j]);
    snapshot_.q_joint[j] = joint.position_rad;
    snapshot_.dq_joint[j] = joint.velocity_rad_s;
  }
}

bool ArmController::validateCalibration(const CalibrationMetadata& cal, std::string* error) const {
  if (cal.calibration_id.empty() || cal.model_hash != config_.model_hash ||
      cal.board_boot_id != config_.board_boot_id || !cal.directions_confirmed ||
      !cal.reference_pose_confirmed || snapshot_.zero_capture_samples < 200)
    return reject(error, "calibration requires matching identity, confirmations and 200 stable BRAKE frames");
  for (std::size_t j = 0; j < kJointCount; ++j) {
    const auto& c = cal.joints[j];
    if (!c.position_calibrated || c.motor_id != config_.motor_ids[j] ||
        (c.direction != -1 && c.direction != 1) ||
        !std::isfinite(c.gear_ratio) || std::abs(c.gear_ratio - config_.gear_ratios[j]) > 1e-9 ||
        !std::isfinite(c.rotor_zero_rad) || !std::isfinite(c.joint_zero_rad) ||
        std::abs(c.rotor_zero_rad - snapshot_.capture_rotor_mean[j]) > config_.capture_span_rotor ||
        c.joint_zero_rad < config_.hard_lower[j] || c.joint_zero_rad > config_.hard_upper[j])
      return reject(error, "calibration mapping or capture reference is invalid");
  }
  return true;
}
bool ArmController::validatePlan(const Plan& plan, std::string* error) const {
  if (plan.plan_id.empty() || plan.model_hash != config_.model_hash ||
      plan.board_boot_id != config_.board_boot_id || plan.calibration_id != snapshot_.calibration_id ||
      !plan.collision_checked || plan.trajectory.size() > 200000)
    return reject(error, "plan requires matching identity, bounded samples and collision attestation");
  try {
    if (plan.trajectory.size() < 2) return reject(error, "plan requires a complete sampled trajectory");
    if (plan.trajectory.front().time_s != 0.0)
      return reject(error, "plan must start at exactly zero seconds");
    qmini_arm::validateHomeTrajectory(plan.trajectory, config_.soft_lower, config_.soft_upper,
        config_.hard_lower, config_.hard_upper, plan.trajectory.back().position_rad, false,
        config_.limit_margin, config_.maximum_velocity, config_.maximum_acceleration,
        config_.maximum_sample_period, config_.maximum_duration, 1e-6);
    for (std::size_t j = 0; j < kJointCount; ++j) {
      if (std::abs(plan.trajectory.front().position_rad[j] - snapshot_.q_joint[j]) > config_.start_tolerance ||
          std::abs(snapshot_.dq_joint[j]) > 0.01)
        return reject(error, "plan start does not match the stopped measured pose");
      for (std::size_t k = 1; k < plan.trajectory.size(); ++k)
        checkSegment(segment(plan.trajectory[k - 1], plan.trajectory[k], j),
                     config_.soft_lower[j] + config_.limit_margin,
                     config_.soft_upper[j] - config_.limit_margin,
                     config_.maximum_velocity, config_.maximum_acceleration);
    }
  } catch (const std::exception& e) { return reject(error, e.what()); }
  return true;
}

bool ArmController::handleCommand(const Command& command, std::string* error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (error) error->clear();
  refresh(clock_());
  expireLease();
  if (command.kind == CommandKind::Estop) {
    fault("emergency stop requested; physical cutoff may be required");
    snapshot_.state = ControllerState::Estop;
    return true;
  }
  if (command.kind == CommandKind::Stop) {
    stopOutput();
    clearPlans();
    if (snapshot_.state != ControllerState::Estop && snapshot_.state != ControllerState::Fault) {
      snapshot_.state = !snapshot_.connected ? ControllerState::Disconnected :
          !snapshot_.position_calibrated ? ControllerState::ReadOnly :
          withinSoftLimits() ? ControllerState::Ready : ControllerState::CalibrationValid;
    }
    return true;
  }
  if (!snapshot_.connected) return reject(error, "controller is disconnected");
  if (command.kind == CommandKind::LeaseHeartbeat) {
    if (command.lease_id.empty() || (!lease_id_.empty() && command.lease_id != lease_id_))
      return reject(error, "lease is missing or owned by another client");
    lease_id_ = command.lease_id;
    lease_deadline_ns_ = clock_() + ns(config_.lease_timeout_s);
    snapshot_.lease_active = true;
    return true;
  }
  if (!requireLease(command, error)) return false;
  if (command.kind == CommandKind::ResetFault) {
    if ((snapshot_.state != ControllerState::Fault && snapshot_.state != ControllerState::Estop) ||
        !command.external_support_confirmed || !feedbackHealthy(error))
      return reject(error, "fault reset requires fault state, support confirmation and healthy feedback");
    stopOutput();
    resetCalibration();
    snapshot_.fault.clear();
    snapshot_.state = ControllerState::ReadOnly;
    return true;
  }
  if (snapshot_.state == ControllerState::Fault || snapshot_.state == ControllerState::Estop)
    return reject(error, "fault is latched; explicit reset and recalibration are required");
  if (!feedbackHealthy(error)) return false;
  if (command.kind == CommandKind::ZeroCapture) {
    if (snapshot_.state != ControllerState::ReadOnly || !command.external_support_confirmed ||
        command.model_hash != config_.model_hash || command.board_boot_id != config_.board_boot_id)
      return reject(error, "capture requires READ_ONLY, support confirmation and current model/boot");
    resetCalibration();
    snapshot_.capture_rotor_mean = {};
    capture_min_.fill(std::numeric_limits<double>::infinity());
    capture_max_.fill(-std::numeric_limits<double>::infinity());
    snapshot_.state = ControllerState::ZeroCapture;
    return true;
  }
  if (command.kind == CommandKind::ZeroCommit) {
    if (snapshot_.state != ControllerState::ZeroCapture ||
        !validateCalibration(command.calibration, error))
      return reject(error, "zero commit requires a valid completed capture");
    calibration_ = command.calibration;
    snapshot_.calibration_id = calibration_.calibration_id;
    snapshot_.position_calibrated = true;
    updateJointState();
    snapshot_.q_des = snapshot_.q_joint;
    snapshot_.state = withinSoftLimits() ? ControllerState::Ready : ControllerState::CalibrationValid;
    return true;
  }
  if (!requireIdentity(command, error)) return false;
  if (command.kind == CommandKind::GravitySet) {
    if ((snapshot_.state != ControllerState::Ready && snapshot_.state != ControllerState::GravityHold) ||
        !std::isfinite(command.gravity_scale) || command.gravity_scale < 0 || command.gravity_scale > 1)
      return reject(error, "gravity requires READY/GRAVITY_HOLD and scale within [0, 1]");
    gravity_target_ = command.gravity_scale;
    if (gravity_target_ > 0) snapshot_.state = ControllerState::GravityHold;
    return true;
  }
  if (snapshot_.state != ControllerState::Ready) return reject(error, "planning requires READY");
  if (command.kind == CommandKind::PlanValidate) {
    if (plans_.count(command.plan.plan_id) || plans_.size() >= 128)
      return reject(error, "plan_id is immutable or the plan cache is full");
    if (!validatePlan(command.plan, error)) return false;
    plans_.emplace(command.plan.plan_id, command.plan);
    return true;
  }
  if (command.kind == CommandKind::PlanExecute) {
    const auto found = plans_.find(command.plan_id);
    if (found == plans_.end() || !command.plan.trajectory.empty())
      return reject(error, "execute accepts only a previously validated immutable plan_id");
    if (!validatePlan(found->second, error)) return false;
    snapshot_.active_plan_id = command.plan_id;
    execution_start_ns_ = clock_();
    snapshot_.state = ControllerState::Executing;
    gravity_target_ = 1.0;
    return true;
  }
  return reject(error, "unsupported command");
}

bool ArmController::updateFeedback(const FeedbackFrame& frame, std::string* error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (error) error->clear();
  const auto now = clock_();
  refresh(now);
  if (!snapshot_.connected) return reject(error, "controller is disconnected");
  if (frame.monotonic_ns > now || now - frame.monotonic_ns >= ns(config_.feedback_timeout_s) ||
      (have_feedback_ && (frame.sequence <= snapshot_.feedback.sequence ||
                         frame.monotonic_ns <= snapshot_.feedback.monotonic_ns))) {
    fault("stale, repeated or nonmonotonic feedback");
    return reject(error, snapshot_.fault);
  }
  const bool contiguous = !have_feedback_ || frame.sequence == snapshot_.feedback.sequence + 1;
  snapshot_.feedback = frame;
  feedback_received_ns_ = frame.monotonic_ns;
  have_feedback_ = true;
  std::string feedback_error;
  if (!feedbackHealthy(&feedback_error)) {
    fault(feedback_error);
    return reject(error, feedback_error);
  }
  if (snapshot_.state == ControllerState::ZeroCapture) {
    if (!contiguous) { fault("zero capture sequence is not contiguous"); return reject(error, snapshot_.fault); }
    for (std::size_t j = 0; j < kJointCount; ++j) {
      const double q = frame.motors[j].position_rad;
      capture_min_[j] = std::min(capture_min_[j], q);
      capture_max_[j] = std::max(capture_max_[j], q);
      if (capture_max_[j] - capture_min_[j] > config_.capture_span_rotor ||
          std::abs(frame.motors[j].velocity_rad_s) > config_.capture_span_rotor) {
        fault("zero capture requires stationary BRAKE feedback");
        return reject(error, snapshot_.fault);
      }
      snapshot_.capture_rotor_mean[j] += (q - snapshot_.capture_rotor_mean[j]) /
          static_cast<double>(snapshot_.zero_capture_samples + 1);
    }
    ++snapshot_.zero_capture_samples;
  }
  updateJointState();
  if (snapshot_.position_calibrated) {
    for (std::size_t j = 0; j < kJointCount; ++j) {
      if (snapshot_.q_joint[j] < config_.hard_lower[j] || snapshot_.q_joint[j] > config_.hard_upper[j]) {
        fault("measured joint pose violates hard limits");
        return reject(error, snapshot_.fault);
      }
    }
  }
  if (snapshot_.state == ControllerState::CalibrationValid && withinSoftLimits())
    snapshot_.state = ControllerState::Ready;
  if ((snapshot_.state == ControllerState::Ready || powered(snapshot_.state)) && !withinSoftLimits()) {
    fault("measured joint pose violates guarded soft limits");
    return reject(error, snapshot_.fault);
  }
  return true;
}

bool ArmController::tick(double dt_s, std::string* error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (error) error->clear();
  const auto now = clock_();
  refresh(now);
  expireLease();
  if (!std::isfinite(dt_s) || dt_s <= 0 || dt_s > config_.maximum_tick_s ||
      (last_tick_ns_ && (now < last_tick_ns_ || now - last_tick_ns_ > ns(config_.maximum_tick_s)))) {
    fault("control cycle watchdog expired or invalid dt");
    return reject(error, snapshot_.fault);
  }
  last_tick_ns_ = now;
  if (snapshot_.state == ControllerState::Fault || snapshot_.state == ControllerState::Estop)
    return reject(error, snapshot_.fault);
  if (!powered(snapshot_.state)) return true;
  std::string feedback_error;
  if (!feedbackHealthy(&feedback_error) || !withinSoftLimits()) {
    fault(feedback_error.empty() ? "unsafe measured joint pose" : feedback_error);
    return reject(error, snapshot_.fault);
  }
  bool finished = false;
  if (snapshot_.state == ControllerState::Executing) {
    const auto& trajectory = plans_.at(snapshot_.active_plan_id).trajectory;
    const double elapsed = static_cast<double>(now - execution_start_ns_) / 1e9;
    if (elapsed >= trajectory.back().time_s) {
      snapshot_.q_des = trajectory.back().position_rad;
      snapshot_.dq_des = {};
      finished = true;
    } else {
      const auto it = std::upper_bound(trajectory.begin(), trajectory.end(), elapsed,
          [](double time, const qmini_arm::JointTrajectorySample& sample) { return time < sample.time_s; });
      const auto& end = *it;
      const auto& start = *(it - 1);
      for (std::size_t j = 0; j < kJointCount; ++j) {
        const auto p = segment(start, end, j);
        const double u = (elapsed - start.time_s) / p.dt;
        snapshot_.q_des[j] = p.q(u);
        snapshot_.dq_des[j] = p.v(u);
      }
    }
    for (std::size_t j = 0; j < kJointCount; ++j) {
      snapshot_.tracking_error[j] = snapshot_.q_des[j] - snapshot_.q_joint[j];
      if (std::abs(snapshot_.tracking_error[j]) >
          (finished ? config_.start_tolerance : config_.tracking_tolerance)) {
        fault("trajectory tracking error exceeds limit");
        return reject(error, snapshot_.fault);
      }
    }
  }
  snapshot_.gravity_scale += clamp(gravity_target_ - snapshot_.gravity_scale,
      -config_.gravity_ramp_per_s * dt_s, config_.gravity_ramp_per_s * dt_s);
  JointArray requested{};
  JointArray previous{};
  const auto gravity = gravity_model_.compensationTorque(snapshot_.q_joint);
  for (std::size_t j = 0; j < kJointCount; ++j) {
    double joint_torque = snapshot_.gravity_scale * gravity[j] -
        config_.velocity_kd[j] * snapshot_.dq_joint[j];
    if (snapshot_.state == ControllerState::Executing) {
      joint_torque += config_.position_kp[j] * snapshot_.tracking_error[j] +
                      config_.velocity_kd[j] * snapshot_.dq_des[j];
    }
    requested[j] = qmini_arm::jointTorqueToRotorNm(joint_torque, calibration_.joints[j]);
    previous[j] = output_.motors[j].torque_ff_nm;
  }
  const auto limited = qmini_arm::limitRotorTorque(requested, previous, config_.rotor_torque_caps,
                                                   config_.rotor_torque_slew_per_s * dt_s, nullptr);
  output_.brake = false;
  for (std::size_t j = 0; j < kJointCount; ++j) {
    auto& out = output_.motors[j];
    out = {};
    out.torque_ff_nm = limited[j];
    snapshot_.tau_ff[j] = limited[j];
    if (snapshot_.state == ControllerState::Executing) {
      out.position_rad = qmini_arm::jointPositionToRotorRad(snapshot_.q_des[j], calibration_.joints[j]);
      out.velocity_rad_s = qmini_arm::jointVelocityToRotorRadS(snapshot_.dq_des[j], calibration_.joints[j]);
      // PD is evaluated above so the cap and slew bound total requested torque.
      // Vendor gains stay zero; q/dq are diagnostics for the ideal fake adapter.
    }
  }
  if (finished) {
    // A completed move returns to gravity support, avoiding a sudden release.
    snapshot_.state = ControllerState::GravityHold;
    snapshot_.active_plan_id.clear();
    for (auto& motor : output_.motors) { motor.kp = 0; motor.kd = 0; motor.velocity_rad_s = 0; }
  }
  if (snapshot_.state == ControllerState::GravityHold && gravity_target_ == 0 && snapshot_.gravity_scale == 0) {
    stopOutput();
    snapshot_.state = ControllerState::Ready;
  }
  return true;
}

ControllerSnapshot ArmController::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}
ControlOutput ArmController::output() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return output_;
}
ControllerConfig ArmController::config() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return config_;
}

FakeController::FakeController(ControllerConfig config)
    : controller_(std::move(config), [this] { return now_ns_; }) { controller_.connect(); }
void FakeController::setRotorPosition(const JointArray& position) { rotor_position_ = position; }
bool FakeController::advance(double dt_s, std::string* error) {
  if (!positive(dt_s) || dt_s > 3600) return false;
  now_ns_ += ns(dt_s);
  const auto output = controller_.output();
  const bool executing = controller_.snapshot().state == ControllerState::Executing;
  FeedbackFrame feedback;
  feedback.sequence = ++sequence_;
  feedback.monotonic_ns = now_ns_;
  for (std::size_t j = 0; j < kJointCount; ++j) {
    auto& motor = feedback.motors[j];
    motor.motor_id = output.motor_ids[j];
    motor.mode = output.brake ? 0 : 1;
    motor.temperature_c = 25;
    if (!output.brake && executing) {
      motor.velocity_rad_s = output.motors[j].velocity_rad_s;
      rotor_position_[j] = output.motors[j].position_rad;
    }
    motor.position_rad = rotor_position_[j];
    motor.torque_estimate_nm = output.motors[j].torque_ff_nm;
    feedback.valid[j] = true;
  }
  const bool accepted = controller_.updateFeedback(feedback, error);
  const bool ticked = controller_.tick(dt_s, error);
  return accepted && ticked;
}
}  // namespace qarm_controller
