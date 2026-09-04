#include "qmini_arm/motor_bus.hpp"

#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <thread>
#include <unistd.h>

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

namespace qmini_arm {

namespace {

void validateMotorId(int motor_id) {
  if (motor_id < 0 || motor_id > 14) {
    throw std::invalid_argument(
        "motor ID must be in [0, 14]; ID 15 is broadcast/no reply");
  }
}

std::string validateSerialPort(const std::string& serial_port) {
  if (serial_port.empty()) {
    throw std::invalid_argument("serial port must not be empty");
  }
  return serial_port;
}

void validateCommand(const MotorCommand& command) {
  if (!std::isfinite(command.torque_ff_nm) ||
      !std::isfinite(command.velocity_rad_s) ||
      !std::isfinite(command.position_rad) || !std::isfinite(command.kp) ||
      !std::isfinite(command.kd)) {
    throw std::invalid_argument("motor command values must be finite");
  }
  if (command.kp < 0.0 || command.kd < 0.0) {
    throw std::invalid_argument("motor command kp/kd must be non-negative");
  }
}

class BusLock {
 public:
  BusLock() {
    fd_ = ::open("/tmp/qarm_m8010_bus.lock", O_CREAT | O_RDWR, 0600);
    if (fd_ < 0 || ::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
      if (fd_ >= 0) ::close(fd_);
      fd_ = -1;
      throw std::runtime_error("another Qarm process owns the motor bus lock");
    }
  }

  ~BusLock() {
    if (fd_ >= 0) {
      ::flock(fd_, LOCK_UN);
      ::close(fd_);
    }
  }

  BusLock(const BusLock&) = delete;
  BusLock& operator=(const BusLock&) = delete;

 private:
  int fd_ = -1;
};

}  // namespace

class MotorBus::Impl {
 public:
  explicit Impl(const std::string& serial_port)
      : bus_lock(),
        kind(MotorType::GO_M8010_6),
        foc_mode(queryMotorMode(kind, MotorMode::FOC)),
        brake_mode(queryMotorMode(kind, MotorMode::BRAKE)),
        gear_ratio(queryGearRatio(kind)),
        serial(serial_port) {}

  BusLock bus_lock;
  MotorType kind;
  int foc_mode;
  int brake_mode;
  double gear_ratio;
  SerialPort serial;
};

MotorBus::MotorBus(const std::string& serial_port)
    : impl_(new Impl(validateSerialPort(serial_port))) {
  if (!std::isfinite(impl_->gear_ratio) || impl_->gear_ratio <= 0.0) {
    throw std::runtime_error("Unitree SDK returned an invalid gear ratio");
  }
}

MotorBus::~MotorBus() = default;

double MotorBus::gearRatio() const { return impl_->gear_ratio; }

int MotorBus::focMode() const { return impl_->foc_mode; }

int MotorBus::brakeMode() const { return impl_->brake_mode; }

MotorState MotorBus::exchange(int motor_id, const MotorCommand& command) {
  return exchangeWithMode(motor_id, command, impl_->foc_mode);
}

MotorState MotorBus::exchangeWithMode(int motor_id,
                                      const MotorCommand& command,
                                      int mode) {
  validateMotorId(motor_id);
  validateCommand(command);

  // MotorCmd's SDK constructor does not initialize every public field, so all
  // fields are deliberately assigned for every exchange.
  MotorCmd sdk_command;
  MotorData sdk_state;
  sdk_command.motorType = impl_->kind;
  sdk_state.motorType = impl_->kind;
  sdk_state.correct = false;
  sdk_command.id = static_cast<unsigned short>(motor_id);
  sdk_command.mode = static_cast<unsigned short>(mode);
  sdk_command.tau = static_cast<float>(command.torque_ff_nm);
  sdk_command.dq = static_cast<float>(command.velocity_rad_s);
  sdk_command.q = static_cast<float>(command.position_rad);
  sdk_command.kp = static_cast<float>(command.kp);
  sdk_command.kd = static_cast<float>(command.kd);

  const bool ok = impl_->serial.sendRecv(&sdk_command, &sdk_state);
  if (!ok || !sdk_state.correct) {
    throw std::runtime_error("motor ID " + std::to_string(motor_id) +
                             ": timeout, short read, or CRC failure");
  }
  if (static_cast<int>(sdk_state.motor_id) != motor_id) {
    throw std::runtime_error(
        "motor ID " + std::to_string(motor_id) +
        ": reply ID mismatch, got " +
        std::to_string(static_cast<int>(sdk_state.motor_id)));
  }

  MotorState result;
  result.motor_id = motor_id;
  result.position_rad = sdk_state.q;
  result.velocity_rad_s = sdk_state.dq;
  result.torque_estimate_nm = sdk_state.tau;
  result.temperature_c = sdk_state.temp;
  result.error_code = sdk_state.merror;
  result.mode = sdk_state.mode;
  return result;
}

MotorState MotorBus::readStateZeroOutput(int motor_id) {
  // All fields default to zero. With kp=0, q=0 does not request a move.
  return exchange(motor_id, MotorCommand{});
}

MotorState MotorBus::readStateBrake(int motor_id) {
  return exchangeWithMode(motor_id, MotorCommand{}, impl_->brake_mode);
}

int MotorBus::sendZeroOutput(const std::vector<int>& motor_ids,
                             int repeat_count) noexcept {
  int acknowledgements = 0;
  if (repeat_count <= 0) return acknowledgements;

  for (int round = 0; round < repeat_count; ++round) {
    for (const int motor_id : motor_ids) {
      try {
        (void)readStateZeroOutput(motor_id);
        ++acknowledgements;
      } catch (const std::exception&) {
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return acknowledgements;
}

int MotorBus::sendBrake(const std::vector<int>& motor_ids,
                        int repeat_count) noexcept {
  int acknowledgements = 0;
  if (repeat_count <= 0) return acknowledgements;

  for (int round = 0; round < repeat_count; ++round) {
    for (const int motor_id : motor_ids) {
      try {
        const MotorState state = readStateBrake(motor_id);
        if (state.mode == impl_->brake_mode && state.error_code == 0 &&
            std::isfinite(state.position_rad) &&
            std::isfinite(state.velocity_rad_s)) {
          ++acknowledgements;
        }
      } catch (const std::exception&) {
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return acknowledgements;
}

}  // namespace qmini_arm
