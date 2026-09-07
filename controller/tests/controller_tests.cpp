#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include "qarm_controller/arm_controller.hpp"

using namespace qarm_controller;
namespace {
void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}
ControllerConfig config() {
  ControllerConfig cfg;
  cfg.model_hash = "model-v1";
  cfg.board_boot_id = "boot-v1";
  return cfg;
}
Command command(CommandKind kind) {
  Command cmd;
  cmd.kind = kind;
  cmd.lease_id = "viser-client";
  cmd.model_hash = "model-v1";
  cmd.board_boot_id = "boot-v1";
  cmd.calibration_id = "cal-v1";
  return cmd;
}
void accepted(ArmController& controller, const Command& cmd) {
  std::string error;
  require(controller.handleCommand(cmd, &error), commandKindName(cmd.kind) + std::string(": ") + error);
}
void advance(FakeController& fake, double dt = 0.01) {
  std::string error;
  require(fake.advance(dt, &error), "fake advance: " + error);
}
CalibrationMetadata calibration(const JointArray& reference = JointArray{}) {
  CalibrationMetadata cal;
  cal.calibration_id = "cal-v1";
  cal.model_hash = "model-v1";
  cal.board_boot_id = "boot-v1";
  cal.directions_confirmed = true;
  cal.reference_pose_confirmed = true;
  for (std::size_t j = 0; j < kJointCount; ++j) {
    cal.joints[j].motor_id = static_cast<int>(j);
    cal.joints[j].position_calibrated = true;
    cal.joints[j].joint_zero_rad = reference[j];
  }
  return cal;
}
void capture(FakeController& fake) {
  auto& controller = fake.controller();
  advance(fake);
  accepted(controller, command(CommandKind::LeaseHeartbeat));
  auto start = command(CommandKind::ZeroCapture);
  start.external_support_confirmed = true;
  accepted(controller, start);
  for (int i = 0; i < 200; ++i) {
    if (i % 50 == 0) accepted(controller, command(CommandKind::LeaseHeartbeat));
    advance(fake);
  }
}
void ready(FakeController& fake, const JointArray& reference = JointArray{}) {
  capture(fake);
  auto commit = command(CommandKind::ZeroCommit);
  commit.calibration = calibration(reference);
  accepted(fake.controller(), commit);
}
Plan plan(const std::string& id = "plan-v1") {
  Plan p;
  p.plan_id = id;
  p.model_hash = "model-v1";
  p.board_boot_id = "boot-v1";
  p.calibration_id = "cal-v1";
  p.collision_checked = true;
  for (int i = 0; i <= 100; ++i) {
    qmini_arm::JointTrajectorySample sample;
    const double t = i / 100.0;
    sample.time_s = t;
    sample.position_rad[0] = 0.05 * (3 * t * t - 2 * t * t * t);
    sample.velocity_rad_s[0] = 0.05 * (6 * t - 6 * t * t);
    p.trajectory.push_back(sample);
  }
  return p;
}
void testUncalibratedAndCapture() {
  FakeController fake(config());
  advance(fake);
  auto& c = fake.controller();
  require(std::isnan(c.snapshot().q_joint[0]), "uncalibrated position must be NaN");
  require(!c.handleCommand(command(CommandKind::GravitySet)), "gravity accepted before calibration");
  accepted(c, command(CommandKind::LeaseHeartbeat));
  auto start = command(CommandKind::ZeroCapture);
  require(!c.handleCommand(start), "capture accepted without support confirmation");
  start.external_support_confirmed = true;
  accepted(c, start);
  auto commit = command(CommandKind::ZeroCommit);
  commit.calibration = calibration();
  require(!c.handleCommand(commit), "capture committed without 200 frames");
  for (int i = 0; i < 200; ++i) {
    if (i % 50 == 0) accepted(c, command(CommandKind::LeaseHeartbeat));
    advance(fake);
  }
  auto invalid = commit;
  invalid.calibration.board_boot_id = "old-boot";
  require(!c.handleCommand(invalid), "stale calibration accepted");
  invalid = commit;
  invalid.calibration.joints[0].direction = 0;
  require(!c.handleCommand(invalid), "invalid direction accepted");
  invalid = commit;
  invalid.calibration.joints[0].motor_id = 1;
  require(!c.handleCommand(invalid), "invalid motor map accepted");
  accepted(c, commit);
  require(c.snapshot().state == ControllerState::Ready, "valid capture did not become ready");
}
void testReferenceOutsideSoftLimits() {
  FakeController fake(config());
  ready(fake, JointArray{{0, 1.5, 0, 0}});
  require(fake.controller().snapshot().state == ControllerState::CalibrationValid,
          "hard-limit reference incorrectly became READY");
  auto gravity = command(CommandKind::GravitySet);
  gravity.gravity_scale = 0.1;
  require(!fake.controller().handleCommand(gravity), "out-of-soft-limit gravity accepted");
  fake.setRotorPosition(JointArray{{0, -0.6 * 6.33, 0, 0}});
  advance(fake);
  require(fake.controller().snapshot().state == ControllerState::Ready,
          "supported return inside soft limits did not become READY");
  require(std::abs(fake.controller().snapshot().q_joint[1] - 0.9) < 1e-9,
          "explicit rotor/joint conversion failed");
}
void testPlanValidationAndExecution() {
  FakeController fake(config());
  ready(fake);
  auto& c = fake.controller();
  auto validate = command(CommandKind::PlanValidate);
  validate.plan = plan();
  auto bad = validate;
  bad.model_hash.clear();
  require(!c.handleCommand(bad), "missing command identity accepted");
  bad = validate;
  bad.plan.trajectory.front().time_s = 1e-10;
  require(!c.handleCommand(bad), "nonzero first timestamp accepted");
  bad = validate;
  bad.plan.collision_checked = false;
  require(!c.handleCommand(bad), "unchecked collision attestation accepted");
  bad = validate;
  bad.plan.trajectory[50].position_rad[0] = 2;
  require(!c.handleCommand(bad), "unsafe trajectory accepted");
  bad = validate;
  bad.plan.trajectory.resize(2);
  bad.plan.trajectory[1].time_s = 0.05;
  bad.plan.trajectory[1].position_rad[0] = 0.0005;
  bad.plan.trajectory[1].velocity_rad_s[0] = 0;
  require(!c.handleCommand(bad), "Hermite acceleration overshoot accepted");
  accepted(c, validate);
  require(!c.handleCommand(validate), "validated immutable ID was overwritten");
  validate.plan.trajectory.back().position_rad[0] = 0.8;
  auto execute = command(CommandKind::PlanExecute);
  execute.plan_id = "unknown";
  require(!c.handleCommand(execute), "unknown plan executed");
  execute.plan_id = "plan-v1";
  auto changed = execute;
  changed.plan = validate.plan;
  require(!c.handleCommand(changed), "execute accepted replacement trajectory");
  accepted(c, execute);
  advance(fake);
  require(c.snapshot().q_des[0] > 0 && c.snapshot().q_des[0] < 0.001,
          "execution jumped directly to target");
  for (int i = 1; i < 101; ++i) {
    if (i % 50 == 0) accepted(c, command(CommandKind::LeaseHeartbeat));
    advance(fake);
  }
  require(c.snapshot().state == ControllerState::GravityHold, "completed plan did not retain gravity support");
  require(std::abs(c.snapshot().q_des[0] - 0.05) < 1e-9, "stored plan changed after validation");
  accepted(c, command(CommandKind::Stop));
  require(c.output().brake && c.snapshot().state == ControllerState::Ready, "stop failed");
}
void testGravityAndLease() {
  FakeController fake(config());
  ready(fake);
  auto& c = fake.controller();
  auto gravity = command(CommandKind::GravitySet);
  gravity.gravity_scale = std::numeric_limits<double>::quiet_NaN();
  require(!c.handleCommand(gravity), "NaN gravity accepted");
  gravity.gravity_scale = 1.1;
  require(!c.handleCommand(gravity), "out-of-range gravity accepted");
  gravity.gravity_scale = 0.1;
  auto stolen = gravity;
  stolen.lease_id = "second-client";
  require(!c.handleCommand(stolen), "second client obtained control");
  accepted(c, gravity);
  advance(fake);
  require(std::abs(c.snapshot().gravity_scale - 0.005) < 1e-12, "gravity ramp missing");
  for (const auto& out : c.output().motors) {
    require(std::abs(out.torque_ff_nm) <= 0.005 + 1e-9, "initial torque slew violated");
    require(out.kp == 0 && out.kd == 0, "vendor gains bypass total torque cap");
  }
  bool expired = false;
  for (int i = 0; i < 200; ++i) {
    if (!fake.advance(0.01)) { expired = true; break; }
  }
  require(expired && c.snapshot().state == ControllerState::Fault && c.output().brake,
          "lease timeout did not fault and remove output");
  require(!c.handleCommand(gravity), "expired lease accepted motion");
}
void testEstopLatchAndReset() {
  FakeController fake(config());
  ready(fake);
  auto& c = fake.controller();
  accepted(c, command(CommandKind::Estop));
  accepted(c, command(CommandKind::Stop));
  c.disconnect();
  c.connect();
  require(c.snapshot().state == ControllerState::Estop, "stop or reconnect cleared ESTOP");
  fake.advance(0.01);
  accepted(c, command(CommandKind::LeaseHeartbeat));
  auto reset = command(CommandKind::ResetFault);
  require(!c.handleCommand(reset), "fault reset accepted without support confirmation");
  reset.external_support_confirmed = true;
  accepted(c, reset);
  require(c.snapshot().state == ControllerState::ReadOnly && !c.snapshot().position_calibrated,
          "reset retained calibration or enabled motion");
}
void testWatchdogsAndFeedback() {
  FakeController fake(config());
  ready(fake);
  auto& c = fake.controller();
  auto gravity = command(CommandKind::GravitySet);
  gravity.gravity_scale = 0.1;
  accepted(c, gravity);
  advance(fake);
  require(!fake.advance(0.1), "late control cycle accepted");
  require(c.output().brake && c.snapshot().state == ControllerState::Fault, "watchdog failed to stop");
  FakeController repeated(config());
  advance(repeated);
  auto frame = repeated.controller().snapshot().feedback;
  require(!repeated.controller().updateFeedback(frame), "replayed feedback accepted");
  require(repeated.controller().snapshot().state == ControllerState::Fault, "replayed feedback did not fault");
  FakeController moving(config());
  capture(moving);
  moving.setRotorPosition(JointArray{{0.1, 0, 0, 0}});
  require(!moving.advance(0.01), "moving calibration reference accepted");
  FakeController stale(config());
  ready(stale);
  auto frame2 = stale.controller().snapshot().feedback;
  frame2.sequence++;
  frame2.monotonic_ns++;
  require(!stale.controller().updateFeedback(frame2), "future feedback accepted");
  std::uint64_t now = 1000000000;
  ArmController bad_motor(config(), [&now] { return now; });
  bad_motor.connect();
  FeedbackFrame frame3;
  frame3.sequence = 1;
  frame3.monotonic_ns = now;
  for (std::size_t j = 0; j < kJointCount; ++j) {
    frame3.valid[j] = true;
    frame3.motors[j].motor_id = static_cast<int>(j);
    frame3.motors[j].temperature_c = 25;
  }
  frame3.motors[0].temperature_c = 60;
  require(!bad_motor.updateFeedback(frame3), "hot feedback accepted");
  require(bad_motor.snapshot().fault.find("temperature") != std::string::npos,
          "temperature fault did not preserve the diagnostic");
}
void testBadConfiguration() {
  auto cfg = config();
  cfg.motor_ids[1] = cfg.motor_ids[0];
  bool threw = false;
  try { ArmController c(cfg); } catch (const std::invalid_argument&) { threw = true; }
  require(threw, "duplicate motor IDs accepted");
  cfg = config();
  cfg.soft_lower[0] = std::numeric_limits<double>::quiet_NaN();
  threw = false;
  try { ArmController c(cfg); } catch (const std::invalid_argument&) { threw = true; }
  require(threw, "NaN soft limit accepted");
}
}  // namespace
int main() {
  const std::vector<std::pair<std::string, std::function<void()>>> tests = {
      {"uncalibrated and capture", testUncalibratedAndCapture},
      {"reference outside soft limits", testReferenceOutsideSoftLimits},
      {"immutable plan and execution", testPlanValidationAndExecution},
      {"gravity and lease", testGravityAndLease},
      {"estop latch and reset", testEstopLatchAndReset},
      {"watchdogs and feedback", testWatchdogsAndFeedback},
      {"invalid configuration", testBadConfiguration}};
  for (const auto& test : tests) {
    try { test.second(); std::cout << "PASS " << test.first << '\n'; }
    catch (const std::exception& e) { std::cerr << "FAIL " << test.first << ": " << e.what() << '\n'; return 1; }
  }
  return 0;
}
