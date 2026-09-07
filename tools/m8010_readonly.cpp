#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

namespace {

std::atomic<bool> running{true};

void stop_handler(int) { running.store(false); }

struct Options {
  std::string device{"/dev/ttyUSB0"};
  std::vector<int> ids{0, 1, 2, 3};
  double rate_hz{100.0};
  std::size_t count{0};
  bool dry_run{false};
  bool acknowledge_state_change{false};
};

std::vector<int> parse_ids(const std::string& value) {
  std::vector<int> result;
  std::stringstream stream(value);
  std::string token;
  while (std::getline(stream, token, ',')) {
    if (token.empty()) throw std::runtime_error("empty motor ID");
    const int id = std::stoi(token);
    if (id < 0 || id > 14) throw std::runtime_error("motor ID outside 0..14");
    for (const int existing : result) {
      if (existing == id) throw std::runtime_error("duplicate motor ID");
    }
    result.push_back(id);
  }
  if (result.empty()) throw std::runtime_error("at least one motor ID is required");
  return result;
}

Options parse_options(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    auto require_value = [&](const char* name) -> std::string {
      if (++i >= argc) throw std::runtime_error(std::string(name) + " requires a value");
      return argv[i];
    };
    if (arg == "--device") {
      options.device = require_value("--device");
    } else if (arg == "--ids") {
      options.ids = parse_ids(require_value("--ids"));
    } else if (arg == "--rate") {
      options.rate_hz = std::stod(require_value("--rate"));
    } else if (arg == "--count") {
      options.count = static_cast<std::size_t>(std::stoull(require_value("--count")));
    } else if (arg == "--mode") {
      const std::string mode = require_value("--mode");
      if (mode != "brake") {
        throw std::runtime_error("only --mode brake is implemented for safety");
      }
    } else if (arg == "--dry-run") {
      options.dry_run = true;
    } else if (arg == "--acknowledge-state-change") {
      options.acknowledge_state_change = true;
    } else if (arg == "--help" || arg == "-h") {
      std::cout
          << "Usage: m8010_readonly [--device PATH] [--ids 0,1,...] "
             "[--rate HZ] [--count N] [--mode brake] [--dry-run] "
             "[--acknowledge-state-change]\n\n"
             "Despite its name, the motor protocol has no passive read. Live "
             "polling sends mode=BRAKE with q=dq=tau=kp=kd=0 before every "
             "reply. This changes motor state. Mechanically support the arm.\n";
      std::exit(0);
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  if (!std::isfinite(options.rate_hz) || options.rate_hz < 1.0 ||
      options.rate_hz > 500.0) {
    throw std::runtime_error("--rate must be within 1..500 Hz");
  }
  return options;
}

class ProcessLock {
 public:
  explicit ProcessLock(const std::string&) {
    path_ = "/tmp/qarm_m8010_bus.lock";
    fd_ = ::open(path_.c_str(), O_CREAT | O_RDWR, 0600);
    if (fd_ < 0) throw std::runtime_error("cannot open process lock " + path_);
    if (::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error("another qarm reader owns " + path_);
    }
  }
  ~ProcessLock() {
    if (fd_ >= 0) {
      ::flock(fd_, LOCK_UN);
      ::close(fd_);
    }
  }
  ProcessLock(const ProcessLock&) = delete;
  ProcessLock& operator=(const ProcessLock&) = delete;

 private:
  int fd_{-1};
  std::string path_;
};

MotorCmd make_brake_command(int id) {
  MotorCmd command;
  command.motorType = MotorType::GO_M8010_6;
  command.hex_len = 0;
  command.id = static_cast<unsigned short>(id);
  command.mode = static_cast<unsigned short>(
      queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE));
  command.tau = 0.0f;
  command.dq = 0.0f;
  command.q = 0.0f;
  command.kp = 0.0f;
  command.kd = 0.0f;
  return command;
}

void reset_feedback(MotorData& data) {
  data.motorType = MotorType::GO_M8010_6;
  data.hex_len = 0;
  data.motor_id = 0xff;
  data.mode = 0xff;
  data.temp = 0;
  data.merror = 0;
  data.tau = 0.0f;
  data.dq = 0.0f;
  data.q = 0.0f;
  data.correct = false;
  data.footForce = 0;
  data.LW = 0.0f;
  data.Acc = 0;
  for (int i = 0; i < 3; ++i) {
    data.gyro[i] = 0.0f;
    data.acc[i] = 0.0f;
  }
}

long long monotonic_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);
    const float ratio = queryGearRatio(MotorType::GO_M8010_6);
    if (!std::isfinite(ratio) || ratio <= 0.0f) {
      throw std::runtime_error("SDK returned an invalid gear ratio");
    }
    std::cout << "{\"type\":\"meta\",\"reader\":\"m8010_readonly\","
              << "\"protocol_is_passive\":false,\"query_mode\":\"BRAKE\","
              << "\"gear_ratio\":" << std::setprecision(9) << ratio
              << ",\"rate_hz\":" << options.rate_hz << ",\"ids\":[";
    for (std::size_t i = 0; i < options.ids.size(); ++i) {
      if (i) std::cout << ',';
      std::cout << options.ids[i];
    }
    std::cout << "]}" << std::endl;
    if (options.dry_run) return 0;
    if (!options.acknowledge_state_change) {
      throw std::runtime_error(
          "live polling changes every motor to BRAKE mode; mechanically support "
          "the arm, clear the workspace, then pass --acknowledge-state-change");
    }

    ProcessLock lock(options.device);
    std::signal(SIGINT, stop_handler);
    std::signal(SIGTERM, stop_handler);
    SerialPort serial(options.device);
    std::vector<MotorCmd> commands;
    commands.reserve(options.ids.size());
    for (const int id : options.ids) commands.push_back(make_brake_command(id));
    std::vector<MotorData> feedback(options.ids.size());
    const auto period = std::chrono::duration<double>(1.0 / options.rate_hz);
    std::size_t sequence = 0;
    auto deadline = std::chrono::steady_clock::now();
    while (running.load() && (options.count == 0 || sequence < options.count)) {
      deadline += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      std::vector<bool> valid(options.ids.size(), false);
      for (std::size_t i = 0; i < options.ids.size(); ++i) {
        reset_feedback(feedback[i]);
        const bool transported = serial.sendRecv(&commands[i], &feedback[i]);
        valid[i] = transported && feedback[i].correct &&
                   static_cast<int>(feedback[i].motor_id) == options.ids[i];
      }
      std::cout << "{\"type\":\"sample\",\"monotonic_ns\":"
                << monotonic_ns() << ",\"sequence\":" << sequence
                << ",\"motors\":[" << std::setprecision(9);
      for (std::size_t i = 0; i < options.ids.size(); ++i) {
        if (i) std::cout << ',';
        const auto& item = feedback[i];
        std::cout << "{\"id\":" << options.ids[i]
                  << ",\"returned_id\":" << static_cast<int>(item.motor_id)
                  << ",\"correct\":" << (valid[i] ? "true" : "false")
                  << ",\"q_sdk_rad\":" << item.q
                  << ",\"dq_sdk_rad_s\":" << item.dq
                  << ",\"q_output_rad\":" << item.q / ratio
                  << ",\"dq_output_rad_s\":" << item.dq / ratio
                  << ",\"tau_sdk_nm\":" << item.tau
                  << ",\"tau_ideal_output_nm\":" << item.tau * ratio
                  << ",\"temperature_c\":" << item.temp
                  << ",\"error\":" << item.merror << '}';
      }
      std::cout << "]}" << std::endl;
      ++sequence;
      std::this_thread::sleep_until(deadline);
      if (std::chrono::steady_clock::now() > deadline + period) {
        deadline = std::chrono::steady_clock::now();
      }
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "m8010_readonly: " << error.what() << std::endl;
    return 2;
  }
}
