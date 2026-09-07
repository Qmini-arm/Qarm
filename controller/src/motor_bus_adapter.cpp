#include "qarm_controller/motor_bus_adapter.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "qmini_arm/motor_bus.hpp"

namespace qarm_controller {
namespace {
std::uint64_t steadyNs() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}
}

MotorBusAdapter::MotorBusAdapter(ArmController* controller,
                                 const std::string& serial_port)
    : controller_(controller) {
  if (!controller_) throw std::invalid_argument("MotorBusAdapter requires a controller");
  const auto cfg = controller_->config();
  motor_ids_.assign(cfg.motor_ids.begin(), cfg.motor_ids.end());
  bus_.reset(new qmini_arm::MotorBus(serial_port));
  for (double ratio : cfg.gear_ratios)
    if (std::abs(ratio - bus_->gearRatio()) > 1e-9)
      throw std::invalid_argument("controller gear ratio differs from the bus motor type");
  if (bus_->focMode() != 1 || bus_->brakeMode() != 0)
    throw std::invalid_argument("unsupported motor mode mapping");
  controller_->connect();
}
MotorBusAdapter::~MotorBusAdapter() { stop(); }

int MotorBusAdapter::stop() noexcept {
  return bus_ ? bus_->sendBrake(motor_ids_) : 0;
}
bool MotorBusAdapter::step(double dt_s, std::string* error) {
  if (error) error->clear();
  try {
    // Check lease, stale feedback and actual cycle delay before any FOC write.
    const bool tick_ok = controller_->tick(dt_s, error);
    const ControlOutput previous = controller_->output();
    FeedbackFrame feedback;
    feedback.sequence = ++sequence_;
    feedback.monotonic_ns = steadyNs();
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      const auto& output = previous.motors[joint];
      feedback.motors[joint] = previous.brake
          ? bus_->readStateBrake(previous.motor_ids[joint])
          : bus_->exchange(previous.motor_ids[joint], output);
      feedback.valid[joint] = true;
    }
    if (!controller_->updateFeedback(feedback, error)) {
      if (stop() < static_cast<int>(kJointCount) && error)
        *error += "; BRAKE acknowledgements incomplete; physical cutoff required";
      return false;
    }
    // Continue BRAKE telemetry while faulted so a later explicit reset can
    // verify fresh healthy feedback. The fault remains latched in the core.
    return tick_ok;
  } catch (const std::exception& e) {
    controller_->reportFault(std::string("motor bus failure: ") + e.what());
    const int acknowledgements = stop();
    if (error) *error = e.what();
    if (acknowledgements < static_cast<int>(kJointCount) && error)
      *error += "; BRAKE acknowledgements incomplete; physical cutoff required";
    return false;
  }
}
}  // namespace qarm_controller
