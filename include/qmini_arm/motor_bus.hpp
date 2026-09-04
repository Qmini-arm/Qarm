#ifndef QMINI_ARM_MOTOR_BUS_HPP_
#define QMINI_ARM_MOTOR_BUS_HPP_

#include <memory>
#include <string>
#include <vector>

#include "qmini_arm/types.hpp"

namespace qmini_arm {

// Thin, typed wrapper around the binary unitree_actuator_sdk. SDK headers are
// hidden behind Impl so future kinematics and control code need not depend on
// vendor-specific structures.
class MotorBus {
 public:
  explicit MotorBus(const std::string& serial_port);
  ~MotorBus();

  MotorBus(const MotorBus&) = delete;
  MotorBus& operator=(const MotorBus&) = delete;

  double gearRatio() const;
  int focMode() const;
  int brakeMode() const;

  // Sends one command and receives the addressed motor's feedback. M8010 is a
  // request-response protocol, so even a state read necessarily sends a command.
  MotorState exchange(int motor_id, const MotorCommand& command);

  // Explicitly named to make the side effect clear: kp=kd=tau=dq=q=0 releases
  // active holding. This is not a passive read and not an emergency stop.
  MotorState readStateZeroOutput(int motor_id);

  // Lowest-motion-risk telemetry request supported by the vendor protocol.
  // It sends mode=BRAKE and five explicit zero fields. This changes motor
  // state and is not a safety-rated mechanical brake.
  MotorState readStateBrake(int motor_id);

  // Returns the number of CRC-valid acknowledgements. It never throws so it can
  // be used during exception unwinding; a short count requires physical cutoff.
  int sendZeroOutput(const std::vector<int>& motor_ids,
                     int repeat_count = 3) noexcept;
  int sendBrake(const std::vector<int>& motor_ids,
                int repeat_count = 3) noexcept;

 private:
  MotorState exchangeWithMode(int motor_id, const MotorCommand& command,
                              int mode);
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace qmini_arm

#endif  // QMINI_ARM_MOTOR_BUS_HPP_
