#ifndef QARM_CONTROLLER_MOTOR_BUS_ADAPTER_HPP_
#define QARM_CONTROLLER_MOTOR_BUS_ADAPTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "qarm_controller/arm_controller.hpp"

namespace qmini_arm {
class MotorBus;
}

namespace qarm_controller {

// It owns one MotorBus, advances the controller from the previous frame,
// sends the resulting output, then accepts each joint's new feedback.
// It is not a transport server and does not accept frontend motor commands.
// Dispatch queued high-level commands on the same owning thread between step()
// calls. Do not mutate the controller from another thread during bus exchange.
class MotorBusAdapter {
 public:
  MotorBusAdapter(ArmController* controller, const std::string& serial_port);
  ~MotorBusAdapter();
  MotorBusAdapter(const MotorBusAdapter&) = delete;
  MotorBusAdapter& operator=(const MotorBusAdapter&) = delete;

  bool step(double dt_s, std::string* error = nullptr);
  int stop() noexcept;

 private:
  ArmController* controller_ = nullptr;
  std::unique_ptr<qmini_arm::MotorBus> bus_;
  std::vector<int> motor_ids_;
  std::uint64_t sequence_ = 0;
};

}  // namespace qarm_controller
#endif
