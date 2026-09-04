#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "cli_utils.hpp"
#include "qmini_arm/joint_conversion.hpp"
#include "qmini_arm/motor_bus.hpp"
#include "qmini_arm/safety.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

volatile std::sig_atomic_t g_stop_requested = 0;

void onSignal(int) { g_stop_requested = 1; }

struct Config {
  std::string port = "/dev/ttyUSB0";
  int motor_id = 0;
  int samples = 1;  // 0 means continuous until Ctrl+C.
  double rate_hz = 10.0;
  int direction = 1;
  bool relative_to_start = false;
  bool has_rotor_zero = false;
  double rotor_zero_rad = 0.0;
  double joint_zero_deg = 0.0;
  bool joint_zero_was_set = false;
  int temperature_limit_c = 60;
  bool assume_yes = false;
};

void printUsage(const char* program) {
  std::cout
      << "Usage: " << program << " [options]\n\n"
      << "Read one GO-M8010-6 state by sending a BRAKE/zero request.\n"
      << "This changes motor state; it is not a passive bus monitor or a "
         "safety-rated mechanical brake.\n\n"
      << "Options (defaults shown):\n"
      << "  --port PATH                 /dev/ttyUSB0\n"
      << "  --id N                      0 (valid: 0..14)\n"
      << "  --samples N                 1; use 0 until Ctrl+C\n"
      << "  --rate-hz HZ                10.0 (0.1..200)\n"
      << "  --direction SIGN            +1 or -1\n"
      << "  --rotor-zero-rad RAD        calibrated rotor angle at joint zero\n"
      << "  --joint-zero-deg DEG        mechanical angle at rotor zero\n"
      << "  --relative-to-start         use first sample as temporary zero\n"
      << "  --temp-limit-c C            60\n"
      << "  --yes                       skip the READ confirmation\n"
      << "  -h, --help                  show this help\n";
}

Config parseArguments(int argc, char** argv) {
  Config config;
  for (int i = 1; i < argc; ++i) {
    const std::string option = argv[i];
    if (option == "-h" || option == "--help") {
      printUsage(argv[0]);
      std::exit(0);
    } else if (option == "--yes") {
      config.assume_yes = true;
    } else if (option == "--relative-to-start") {
      config.relative_to_start = true;
    } else if (option == "--port") {
      config.port = qmini_arm::cli::requireValue(argc, argv, i, option);
    } else if (option == "--id") {
      config.motor_id = qmini_arm::cli::parseInt(
          option, qmini_arm::cli::requireValue(argc, argv, i, option));
    } else if (option == "--samples") {
      config.samples = qmini_arm::cli::parseInt(
          option, qmini_arm::cli::requireValue(argc, argv, i, option));
    } else if (option == "--rate-hz") {
      config.rate_hz = qmini_arm::cli::parseDouble(
          option, qmini_arm::cli::requireValue(argc, argv, i, option));
    } else if (option == "--direction") {
      config.direction = qmini_arm::cli::parseInt(
          option, qmini_arm::cli::requireValue(argc, argv, i, option));
    } else if (option == "--rotor-zero-rad") {
      config.rotor_zero_rad = qmini_arm::cli::parseDouble(
          option, qmini_arm::cli::requireValue(argc, argv, i, option));
      config.has_rotor_zero = true;
    } else if (option == "--joint-zero-deg") {
      config.joint_zero_deg = qmini_arm::cli::parseDouble(
          option, qmini_arm::cli::requireValue(argc, argv, i, option));
      config.joint_zero_was_set = true;
    } else if (option == "--temp-limit-c") {
      config.temperature_limit_c = qmini_arm::cli::parseInt(
          option, qmini_arm::cli::requireValue(argc, argv, i, option));
    } else {
      throw std::runtime_error("unknown option: " + option);
    }
  }
  return config;
}

void validateConfig(const Config& config) {
  if (config.port.empty()) {
    throw std::runtime_error("--port must not be empty");
  }
  qmini_arm::cli::validateIds({config.motor_id});
  if (config.samples < 0 || config.samples > 1000000) {
    throw std::runtime_error("--samples must be in [0, 1000000]");
  }
  if (!(config.rate_hz >= 0.1 && config.rate_hz <= 200.0)) {
    throw std::runtime_error("--rate-hz must be in [0.1, 200]");
  }
  if (config.direction != -1 && config.direction != 1) {
    throw std::runtime_error("--direction must be +1 or -1");
  }
  if (config.relative_to_start && config.has_rotor_zero) {
    throw std::runtime_error(
        "--relative-to-start and --rotor-zero-rad cannot be used together");
  }
  if (config.joint_zero_was_set && !config.relative_to_start &&
      !config.has_rotor_zero) {
    throw std::runtime_error(
        "--joint-zero-deg requires --rotor-zero-rad or --relative-to-start");
  }
  if (config.temperature_limit_c < 30 ||
      config.temperature_limit_c > 85) {
    throw std::runtime_error("--temp-limit-c must be in [30, 85]");
  }
}

void run(const Config& config) {
  using Clock = std::chrono::steady_clock;
  std::unique_ptr<qmini_arm::MotorBus> bus;
  const std::vector<int> ids = {config.motor_id};

  try {
    bus.reset(new qmini_arm::MotorBus(config.port));
    qmini_arm::JointCalibration calibration;
    calibration.motor_id = config.motor_id;
    calibration.direction = config.direction;
    calibration.gear_ratio = bus->gearRatio();
    calibration.rotor_zero_rad = config.rotor_zero_rad;
    calibration.joint_zero_rad = config.joint_zero_deg * kDegToRad;
    calibration.position_calibrated = config.has_rotor_zero;

    if (!config.has_rotor_zero && !config.relative_to_start) {
      std::cerr
          << "NOTE: no joint zero supplied; joint_position_deg will be nan. "
             "q_output_raw_deg is diagnostic only, not an absolute joint "
             "angle.\n";
    }

    std::cout
        << "time_s,motor_id,q_rotor_rad,dq_rotor_rad_s,tau_rotor_est_nm,"
           "q_output_raw_deg,joint_position_deg,joint_velocity_rad_s,"
           "joint_tau_ideal_nm,temp_c,merror,mode,exchange_ms\n";

    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / config.rate_hz));
    const auto started = Clock::now();
    auto deadline = started;
    int sample_index = 0;

    while (!g_stop_requested &&
           (config.samples == 0 || sample_index < config.samples)) {
      const auto cycle_started = Clock::now();
      const auto exchange_started = Clock::now();
      const qmini_arm::MotorState motor_state =
          bus->readStateBrake(config.motor_id);
      const double exchange_ms =
          std::chrono::duration<double, std::milli>(Clock::now() -
                                                    exchange_started)
              .count();
      qmini_arm::validateBasicState(motor_state, bus->brakeMode(),
                                    config.temperature_limit_c);

      if (config.relative_to_start && sample_index == 0) {
        calibration.rotor_zero_rad = motor_state.position_rad;
        calibration.position_calibrated = true;
      }
      const qmini_arm::JointState joint_state =
          qmini_arm::toJointState(motor_state, calibration);
      const double raw_output_deg =
          qmini_arm::rawOutputPositionRad(motor_state, bus->gearRatio()) *
          kRadToDeg;
      const double joint_position_deg =
          joint_state.position_rad * kRadToDeg;
      const double elapsed_s =
          std::chrono::duration<double>(cycle_started - started).count();

      std::cout << std::fixed << std::setprecision(6) << elapsed_s << ','
                << motor_state.motor_id << ',' << motor_state.position_rad << ','
                << motor_state.velocity_rad_s << ','
                << motor_state.torque_estimate_nm << ',' << raw_output_deg << ','
                << joint_position_deg << ',' << joint_state.velocity_rad_s << ','
                << joint_state.torque_estimate_nm << ','
                << motor_state.temperature_c << ',' << motor_state.error_code
                << ',' << motor_state.mode << ',' << exchange_ms << '\n';

      ++sample_index;
      deadline += period;
      std::this_thread::sleep_until(deadline);
    }
  } catch (...) {
    if (bus) {
      const int acknowledged = bus->sendBrake(ids);
      std::cerr << "EXIT BRAKE acknowledgements: " << acknowledged
                << "/3\n";
    }
    throw;
  }

  if (bus) {
    const int acknowledged = bus->sendBrake(ids);
    std::cerr << "EXIT BRAKE acknowledgements: " << acknowledged
              << "/3\n";
    if (acknowledged != 3) {
      std::cerr << "WARNING: final zero-output was not fully confirmed; cut "
                   "motor power.\n";
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Config config = parseArguments(argc, argv);
    validateConfig(config);

    std::cerr
        << "WARNING: reading M8010 state sends a BRAKE/zero request and "
           "changes motor state. It is not a mechanical brake.\n"
        << "Secure the mechanism and provide a physical power cutoff.\n";
    if (!config.assume_yes) {
      std::cerr << "Type READ to open " << config.port << " and query motor ID "
                << config.motor_id << ": " << std::flush;
      std::string confirmation;
      std::getline(std::cin, confirmation);
      if (confirmation != "READ") {
        std::cerr << "Cancelled.\n";
        return 2;
      }
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    run(config);
    return g_stop_requested ? 130 : 0;
  } catch (const std::exception& error) {
    std::cerr << "FAILED: " << error.what() << '\n';
    return 1;
  }
}
