#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "qmini_arm/gravity_model.hpp"
#include "qmini_arm/gravity_control.hpp"
#include "qmini_arm/motor_bus.hpp"
#include "qmini_arm/types.hpp"

namespace {

using Clock = std::chrono::steady_clock;
using qmini_arm::JointVector;

volatile std::sig_atomic_t stop_requested = 0;
volatile std::sig_atomic_t stop_signal = 0;

void onSignal(int signal_number) {
  stop_signal = signal_number;
  stop_requested = 1;
}

void flushOutputOrThrow() {
  std::cout << std::flush;
  if (!std::cout.good()) {
    throw std::runtime_error("controller output stream disconnected");
  }
}

std::string trim(const std::string& value) {
  const std::string whitespace = " \t\r\n";
  const std::size_t first = value.find_first_not_of(whitespace);
  if (first == std::string::npos) return "";
  const std::size_t last = value.find_last_not_of(whitespace);
  return value.substr(first, last - first + 1);
}

std::vector<std::string> split(const std::string& value, char delimiter) {
  std::vector<std::string> result;
  std::stringstream stream(value);
  std::string item;
  while (std::getline(stream, item, delimiter)) result.push_back(trim(item));
  return result;
}

double parseDouble(const std::string& name, const std::string& value) {
  std::size_t used = 0;
  const double result = std::stod(value, &used);
  if (used != value.size() || !std::isfinite(result)) {
    throw std::runtime_error(name + " must be a finite number");
  }
  return result;
}

int parseInt(const std::string& name, const std::string& value) {
  std::size_t used = 0;
  const int result = std::stoi(value, &used);
  if (used != value.size()) throw std::runtime_error(name + " must be an integer");
  return result;
}

template <typename T, typename Parser>
std::array<T, 6> parseArray(const std::string& name,
                            const std::string& value,
                            Parser parser) {
  const std::vector<std::string> items = split(value, ',');
  if (items.size() != 6) {
    throw std::runtime_error(name + " must contain exactly six values");
  }
  std::array<T, 6> result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    result[index] = parser(name, items[index]);
  }
  return result;
}

struct ControllerConfig {
  std::string device;
  std::string expected_board_boot_id;
  double gear_ratio = 0.0;
  std::array<int, 6> motor_ids{};
  std::array<int, 6> directions{};
  JointVector rotor_at_reference_rad{};
  JointVector reference_joint_rad{};
  JointVector soft_lower_rad{};
  JointVector soft_upper_rad{};
  JointVector hard_lower_rad{};
  JointVector hard_upper_rad{};
  double startup_limit_margin_rad = 0.0;
  double runtime_limit_margin_rad = 0.0;
  double max_compensation_scale = 0.0;
  JointVector rotor_torque_caps_nm{};
  double rotor_torque_slew_nm_per_cycle = 0.0;
  double kd_rotor = 0.0;
  JointVector joint_speed_trip_rad_s{};
  JointVector joint_speed_hard_trip_rad_s{};
  int joint_speed_trip_consecutive_cycles = 0;
  double rotor_feedback_torque_trip_nm = 0.0;
  int temperature_trip_c = 0;
  double control_rate_hz = 0.0;
  double loop_deadline_trip_s = 0.0;
  double default_ramp_s = 0.0;
  double default_duration_s = 0.0;
  double maximum_duration_s = 0.0;
};

std::map<std::string, std::string> readKeyValues(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot open config: " + path);
  std::map<std::string, std::string> result;
  std::string line;
  int line_number = 0;
  while (std::getline(stream, line)) {
    ++line_number;
    const std::size_t comment = line.find('#');
    if (comment != std::string::npos) line.erase(comment);
    line = trim(line);
    if (line.empty()) continue;
    const std::size_t equals = line.find('=');
    if (equals == std::string::npos) {
      throw std::runtime_error("config line " + std::to_string(line_number) +
                               " has no '='");
    }
    const std::string key = trim(line.substr(0, equals));
    const std::string value = trim(line.substr(equals + 1));
    if (key.empty() || value.empty() || result.count(key)) {
      throw std::runtime_error("invalid or duplicate config key at line " +
                               std::to_string(line_number));
    }
    result[key] = value;
  }
  return result;
}

std::string required(const std::map<std::string, std::string>& values,
                     const std::string& key) {
  const auto found = values.find(key);
  if (found == values.end()) throw std::runtime_error("missing config key: " + key);
  return found->second;
}

ControllerConfig loadConfig(const std::string& path) {
  const auto values = readKeyValues(path);
  const std::vector<std::string> allowed_keys = {
      "schema_version",
      "device",
      "expected_board_boot_id",
      "gear_ratio",
      "motor_ids",
      "directions",
      "rotor_at_reference_rad",
      "reference_joint_rad",
      "soft_lower_rad",
      "soft_upper_rad",
      "hard_lower_rad",
      "hard_upper_rad",
      "startup_limit_margin_rad",
      "runtime_limit_margin_rad",
      "max_compensation_scale",
      "rotor_torque_caps_nm",
      "rotor_torque_slew_nm_per_cycle",
      "kd_rotor",
      "joint_speed_trip_rad_s",
      "joint_speed_hard_trip_rad_s",
      "joint_speed_trip_consecutive_cycles",
      "rotor_feedback_torque_trip_nm",
      "temperature_trip_c",
      "control_rate_hz",
      "loop_deadline_trip_s",
      "default_ramp_s",
      "default_duration_s",
      "maximum_duration_s",
  };
  for (const auto& item : values) {
    if (std::find(allowed_keys.begin(), allowed_keys.end(), item.first) ==
        allowed_keys.end()) {
      throw std::runtime_error("unknown config key: " + item.first);
    }
  }
  if (parseInt("schema_version", required(values, "schema_version")) != 2) {
    throw std::runtime_error("unsupported gravity config schema_version");
  }
  ControllerConfig config;
  config.device = required(values, "device");
  config.expected_board_boot_id = required(values, "expected_board_boot_id");
  config.gear_ratio = parseDouble("gear_ratio", required(values, "gear_ratio"));
  config.motor_ids = parseArray<int>(
      "motor_ids", required(values, "motor_ids"), parseInt);
  config.directions = parseArray<int>(
      "directions", required(values, "directions"), parseInt);
  config.rotor_at_reference_rad = parseArray<double>(
      "rotor_at_reference_rad", required(values, "rotor_at_reference_rad"),
      parseDouble);
  config.reference_joint_rad = parseArray<double>(
      "reference_joint_rad", required(values, "reference_joint_rad"), parseDouble);
  config.soft_lower_rad = parseArray<double>(
      "soft_lower_rad", required(values, "soft_lower_rad"), parseDouble);
  config.soft_upper_rad = parseArray<double>(
      "soft_upper_rad", required(values, "soft_upper_rad"), parseDouble);
  config.hard_lower_rad = parseArray<double>(
      "hard_lower_rad", required(values, "hard_lower_rad"), parseDouble);
  config.hard_upper_rad = parseArray<double>(
      "hard_upper_rad", required(values, "hard_upper_rad"), parseDouble);
  config.startup_limit_margin_rad = parseDouble(
      "startup_limit_margin_rad", required(values, "startup_limit_margin_rad"));
  config.runtime_limit_margin_rad = parseDouble(
      "runtime_limit_margin_rad", required(values, "runtime_limit_margin_rad"));
  config.max_compensation_scale = parseDouble(
      "max_compensation_scale", required(values, "max_compensation_scale"));
  config.rotor_torque_caps_nm = parseArray<double>(
      "rotor_torque_caps_nm", required(values, "rotor_torque_caps_nm"),
      parseDouble);
  config.rotor_torque_slew_nm_per_cycle = parseDouble(
      "rotor_torque_slew_nm_per_cycle",
      required(values, "rotor_torque_slew_nm_per_cycle"));
  config.kd_rotor = parseDouble("kd_rotor", required(values, "kd_rotor"));
  config.joint_speed_trip_rad_s = parseArray<double>(
      "joint_speed_trip_rad_s", required(values, "joint_speed_trip_rad_s"),
      parseDouble);
  config.joint_speed_hard_trip_rad_s = parseArray<double>(
      "joint_speed_hard_trip_rad_s",
      required(values, "joint_speed_hard_trip_rad_s"), parseDouble);
  config.joint_speed_trip_consecutive_cycles = parseInt(
      "joint_speed_trip_consecutive_cycles",
      required(values, "joint_speed_trip_consecutive_cycles"));
  config.rotor_feedback_torque_trip_nm = parseDouble(
      "rotor_feedback_torque_trip_nm",
      required(values, "rotor_feedback_torque_trip_nm"));
  config.temperature_trip_c = parseInt(
      "temperature_trip_c", required(values, "temperature_trip_c"));
  config.control_rate_hz = parseDouble(
      "control_rate_hz", required(values, "control_rate_hz"));
  config.loop_deadline_trip_s = parseDouble(
      "loop_deadline_trip_s", required(values, "loop_deadline_trip_s"));
  config.default_ramp_s = parseDouble(
      "default_ramp_s", required(values, "default_ramp_s"));
  config.default_duration_s = parseDouble(
      "default_duration_s", required(values, "default_duration_s"));
  config.maximum_duration_s = parseDouble(
      "maximum_duration_s", required(values, "maximum_duration_s"));

  if (config.gear_ratio <= 0.0 || config.max_compensation_scale <= 0.0 ||
      config.max_compensation_scale > 1.0 || config.control_rate_hz < 20.0 ||
      config.control_rate_hz > 200.0 || config.temperature_trip_c > 60 ||
      config.temperature_trip_c < 30 ||
      config.runtime_limit_margin_rad <= 0.0 ||
      config.startup_limit_margin_rad < config.runtime_limit_margin_rad ||
      config.startup_limit_margin_rad > 0.3 || config.kd_rotor < 0.0 ||
      config.kd_rotor > 0.05 ||
      config.rotor_feedback_torque_trip_nm <= 0.0 ||
      config.rotor_feedback_torque_trip_nm > 10.0 ||
      config.rotor_torque_slew_nm_per_cycle <= 0.0 ||
      config.rotor_torque_slew_nm_per_cycle > 0.05 ||
      config.joint_speed_trip_consecutive_cycles < 2 ||
      config.joint_speed_trip_consecutive_cycles > 5 ||
      config.loop_deadline_trip_s < 1.0 / config.control_rate_hz ||
      config.loop_deadline_trip_s > 0.1 || config.default_ramp_s < 2.0 ||
      config.default_duration_s < 2.0 * config.default_ramp_s ||
      config.maximum_duration_s < config.default_duration_s ||
      config.maximum_duration_s > 60.0) {
    throw std::runtime_error("gravity config violates safety bounds");
  }
  for (std::size_t index = 0; index < 6; ++index) {
    if (config.motor_ids[index] != static_cast<int>(index)) {
      throw std::runtime_error("deployment requires motor IDs 0..5 in order");
    }
    if (config.directions[index] != -1 && config.directions[index] != 1) {
      throw std::runtime_error("directions must contain only +1 or -1");
    }
    if (config.soft_lower_rad[index] >= config.soft_upper_rad[index] ||
        config.hard_lower_rad[index] >= config.soft_lower_rad[index] ||
        config.hard_upper_rad[index] <= config.soft_upper_rad[index] ||
        config.rotor_torque_caps_nm[index] <= 0.0 ||
        config.rotor_torque_caps_nm[index] > 2.0 ||
        config.joint_speed_trip_rad_s[index] <= 0.0 ||
        config.joint_speed_trip_rad_s[index] > 1.5 ||
        config.joint_speed_hard_trip_rad_s[index] <=
            config.joint_speed_trip_rad_s[index] ||
        config.joint_speed_hard_trip_rad_s[index] > 2.0) {
      throw std::runtime_error(
          "invalid per-joint limit, torque cap, or speed trip");
    }
  }
  return config;
}

enum class Mode { kDryRun, kShadow, kFoc };

struct Options {
  std::string config_path = "config/gravity_comp.conf";
  Mode mode = Mode::kDryRun;
  bool mode_was_set = false;
  double scale = -1.0;
  double duration_s = -1.0;
  double ramp_s = -1.0;
  bool acknowledge_supported_arm = false;
  bool acknowledge_estop_ready = false;
  bool confirm_same_motor_power_cycle = false;
};

void printUsage(const char* program) {
  std::cout
      << "Usage: " << program << " [--config FILE] MODE [options]\n\n"
      << "Modes (choose one):\n"
      << "  --dry-run                   no serial access; inspect model/config\n"
      << "  --shadow                    BRAKE polling; compute but never send torque\n"
      << "  --enable-foc                send guarded gravity torque (DANGEROUS)\n\n"
      << "FOC options:\n"
      << "  --scale VALUE               required; 0 < value <= configured maximum\n"
      << "  --duration-s SECONDS        bounded run, default from config\n"
      << "  --ramp-s SECONDS            compensation ramp, default from config\n"
      << "  --acknowledge-supported-arm\n"
      << "  --acknowledge-estop-ready\n"
      << "  --confirm-same-motor-power-cycle\n";
}

Options parseOptions(int argc, char** argv) {
  Options result;
  auto set_mode = [&](Mode mode) {
    if (result.mode_was_set) throw std::runtime_error("choose exactly one mode");
    result.mode = mode;
    result.mode_was_set = true;
  };
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    auto value = [&]() -> std::string {
      if (++index >= argc) throw std::runtime_error(option + " requires a value");
      return argv[index];
    };
    if (option == "-h" || option == "--help") {
      printUsage(argv[0]);
      std::exit(0);
    } else if (option == "--config") {
      result.config_path = value();
    } else if (option == "--dry-run") {
      set_mode(Mode::kDryRun);
    } else if (option == "--shadow") {
      set_mode(Mode::kShadow);
    } else if (option == "--enable-foc") {
      set_mode(Mode::kFoc);
    } else if (option == "--scale") {
      result.scale = parseDouble("--scale", value());
    } else if (option == "--duration-s") {
      result.duration_s = parseDouble("--duration-s", value());
    } else if (option == "--ramp-s") {
      result.ramp_s = parseDouble("--ramp-s", value());
    } else if (option == "--acknowledge-supported-arm") {
      result.acknowledge_supported_arm = true;
    } else if (option == "--acknowledge-estop-ready") {
      result.acknowledge_estop_ready = true;
    } else if (option == "--confirm-same-motor-power-cycle") {
      result.confirm_same_motor_power_cycle = true;
    } else {
      throw std::runtime_error("unknown option: " + option);
    }
  }
  if (!result.mode_was_set) {
    throw std::runtime_error("choose --dry-run, --shadow, or --enable-foc");
  }
  return result;
}

std::string readSingleLine(const std::string& path) {
  std::ifstream stream(path);
  std::string value;
  if (!stream || !std::getline(stream, value)) {
    throw std::runtime_error("cannot read " + path);
  }
  return trim(value);
}

JointVector jointPosition(const std::array<qmini_arm::MotorState, 6>& state,
                          const ControllerConfig& config) {
  JointVector result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    result[index] = config.reference_joint_rad[index] +
                    config.directions[index] *
                        (state[index].position_rad -
                         config.rotor_at_reference_rad[index]) /
                        config.gear_ratio;
  }
  return result;
}

JointVector jointVelocity(const std::array<qmini_arm::MotorState, 6>& state,
                          const ControllerConfig& config) {
  JointVector result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    result[index] = config.directions[index] * state[index].velocity_rad_s /
                    config.gear_ratio;
  }
  return result;
}

void validateFeedback(const qmini_arm::MotorState& state,
                      int expected_id,
                      int expected_mode,
                      const ControllerConfig& config) {
  if (state.motor_id != expected_id || state.mode != expected_mode) {
    throw std::runtime_error("motor " + std::to_string(expected_id) +
                             " returned an unexpected ID or mode");
  }
  if (state.error_code != 0) {
    throw std::runtime_error("motor " + std::to_string(expected_id) +
                             " error code " +
                             std::to_string(state.error_code));
  }
  if (state.temperature_c >= config.temperature_trip_c) {
    throw std::runtime_error("motor " + std::to_string(expected_id) +
                             " temperature trip");
  }
  if (!std::isfinite(state.position_rad) ||
      !std::isfinite(state.velocity_rad_s) ||
      !std::isfinite(state.torque_estimate_nm) ||
      std::abs(state.torque_estimate_nm) >
          config.rotor_feedback_torque_trip_nm) {
    throw std::runtime_error("motor " + std::to_string(expected_id) +
                             " invalid or excessive feedback");
  }
}

void validatePoseLimits(const JointVector& position,
                        const ControllerConfig& config,
                        double margin) {
  for (std::size_t index = 0; index < position.size(); ++index) {
    if (position[index] <= config.soft_lower_rad[index] + margin ||
        position[index] >= config.soft_upper_rad[index] - margin) {
      std::ostringstream message;
      message << "joint_" << index + 1 << " at " << position[index]
              << " rad lacks the required " << margin
              << " rad soft-limit margin";
      throw std::runtime_error(message.str());
    }
  }
}

std::array<qmini_arm::MotorState, 6> readBrake(
    qmini_arm::MotorBus& bus, const ControllerConfig& config) {
  std::array<qmini_arm::MotorState, 6> result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    result[index] = bus.readStateBrake(config.motor_ids[index]);
    validateFeedback(result[index], config.motor_ids[index], bus.brakeMode(),
                     config);
  }
  return result;
}

void printVector(const char* name, const JointVector& values) {
  std::cout << name << "=[";
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index) std::cout << ',';
    std::cout << std::fixed << std::setprecision(6) << values[index];
  }
  std::cout << "]";
}

void finalBrake(qmini_arm::MotorBus& bus, const ControllerConfig& config) {
  const std::vector<int> ids(config.motor_ids.begin(), config.motor_ids.end());
  const int acknowledgements = bus.sendBrake(ids, 3);
  std::cerr << "BRAKE acknowledgements=" << acknowledgements << "/18\n";
  if (acknowledgements != 18) {
    std::cerr << "WARNING: BRAKE was not confirmed by every motor; use the "
                 "physical power cutoff.\n";
  }
}

void dryRun(const ControllerConfig& config) {
  qmini_arm::GravityModel model;
  const JointVector zero{};
  const JointVector zero_torque = model.compensationTorque(zero);
  const JointVector reference_torque =
      model.compensationTorque(config.reference_joint_rad);
  std::cout << "dry-run: no serial access\n";
  printVector("q_zero", zero);
  std::cout << ' ';
  printVector("gravity_joint_nm", zero_torque);
  std::cout << '\n';
  printVector("q_reference", config.reference_joint_rad);
  std::cout << ' ';
  printVector("gravity_joint_nm", reference_torque);
  std::cout << '\n';
  std::cout << "max_compensation_scale=" << config.max_compensation_scale << ' ';
  printVector("rotor_torque_caps_nm", config.rotor_torque_caps_nm);
  std::cout << ' ';
  printVector("joint_speed_trip_rad_s", config.joint_speed_trip_rad_s);
  std::cout << ' ';
  printVector("joint_speed_hard_trip_rad_s",
              config.joint_speed_hard_trip_rad_s);
  std::cout << " joint_speed_trip_consecutive_cycles="
            << config.joint_speed_trip_consecutive_cycles;
  std::cout << '\n';
}

int runHardware(const ControllerConfig& config, Options options) {
  if (!options.acknowledge_supported_arm ||
      !options.confirm_same_motor_power_cycle) {
    throw std::runtime_error(
        "hardware modes require --acknowledge-supported-arm and "
        "--confirm-same-motor-power-cycle");
  }
  if (options.mode == Mode::kFoc && !options.acknowledge_estop_ready) {
    throw std::runtime_error("FOC requires --acknowledge-estop-ready");
  }
  if (options.mode == Mode::kFoc && options.scale < 0.0) {
    throw std::runtime_error("FOC requires an explicit --scale");
  }
  if (options.scale < 0.0) options.scale = config.max_compensation_scale;
  if (options.duration_s < 0.0) options.duration_s = config.default_duration_s;
  if (options.ramp_s < 0.0) options.ramp_s = config.default_ramp_s;
  if (options.scale <= 0.0 || options.scale > config.max_compensation_scale ||
      options.duration_s <= 0.0 ||
      options.duration_s > config.maximum_duration_s || options.ramp_s < 2.0 ||
      options.duration_s < 2.0 * options.ramp_s) {
    throw std::runtime_error("scale, duration, or ramp violates safety bounds");
  }
  const std::string boot_id =
      readSingleLine("/proc/sys/kernel/random/boot_id");
  if (boot_id != config.expected_board_boot_id) {
    throw std::runtime_error(
        "board boot ID differs from calibration; recapture zero");
  }

  std::unique_ptr<qmini_arm::MotorBus> bus;
  try {
    bus.reset(new qmini_arm::MotorBus(config.device));
    if (std::abs(bus->gearRatio() - config.gear_ratio) > 1e-3) {
      throw std::runtime_error("configured gear ratio differs from Unitree SDK");
    }
    std::array<qmini_arm::MotorState, 6> states{};
    qmini_arm::PerJointSpeedGuard speed_guard(
        config.joint_speed_trip_rad_s,
        config.joint_speed_hard_trip_rad_s,
        static_cast<std::size_t>(config.joint_speed_trip_consecutive_cycles));
    JointVector position{};
    JointVector velocity{};
    // Require several coherent all-motor BRAKE frames before FOC is possible.
    for (int sample = 0; sample < 5; ++sample) {
      states = readBrake(*bus, config);
      position = jointPosition(states, config);
      velocity = jointVelocity(states, config);
      validatePoseLimits(position, config, config.startup_limit_margin_rad);
      speed_guard.observeFrame(velocity);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    speed_guard.reset();

    qmini_arm::GravityModel gravity;
    JointVector previous_rotor_command{};
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / config.control_rate_hz));
    const auto started = Clock::now();
    auto deadline = started;
    std::size_t cycle = 0;
    std::size_t minor_deadline_misses = 0;
    double maximum_lateness_s = 0.0;
    std::cout << "mode="
              << (options.mode == Mode::kShadow ? "shadow" : "FOC")
              << " scale=" << options.scale << " duration_s="
              << options.duration_s << " ramp_s=" << options.ramp_s << '\n';
    printVector("rotor_torque_caps_nm", config.rotor_torque_caps_nm);
    std::cout << ' ';
    printVector("joint_speed_trip_rad_s", config.joint_speed_trip_rad_s);
    std::cout << ' ';
    printVector("joint_speed_hard_trip_rad_s",
                config.joint_speed_hard_trip_rad_s);
    std::cout << " joint_speed_trip_consecutive_cycles="
              << config.joint_speed_trip_consecutive_cycles;
    std::cout << '\n';
    flushOutputOrThrow();

    while (!stop_requested) {
      const auto cycle_started = Clock::now();
      const double lateness_s =
          std::chrono::duration<double>(cycle_started - deadline).count();
      maximum_lateness_s = std::max(maximum_lateness_s, lateness_s);
      if (cycle > 0 && lateness_s > config.loop_deadline_trip_s) {
        throw std::runtime_error(
            "control loop lateness exceeded the configured watchdog");
      }
      if (cycle > 0 && lateness_s > 1.0 / config.control_rate_hz) {
        ++minor_deadline_misses;
      }
      const double elapsed =
          std::chrono::duration<double>(cycle_started - started).count();
      if (elapsed >= options.duration_s) break;
      position = jointPosition(states, config);
      velocity = jointVelocity(states, config);
      validatePoseLimits(position, config, config.runtime_limit_margin_rad);
      speed_guard.observeFrame(velocity);
      const JointVector joint_torque = gravity.compensationTorque(position);
      const double ramp = qmini_arm::symmetricRampEnvelope(
          elapsed, options.duration_s, options.ramp_s);
      const JointVector requested_rotor =
          qmini_arm::jointGravityToRotorTorque(
              joint_torque, ramp * options.scale, config.directions,
              config.gear_ratio);
      bool saturated = false;
      const JointVector rotor_command = qmini_arm::limitRotorTorque(
          requested_rotor, previous_rotor_command,
          config.rotor_torque_caps_nm,
          config.rotor_torque_slew_nm_per_cycle, &saturated);

      std::array<qmini_arm::MotorState, 6> next{};
      for (std::size_t index = 0; index < next.size(); ++index) {
        if (stop_requested) break;
        const double send_lateness_s = std::chrono::duration<double>(
                                           Clock::now() - deadline)
                                           .count();
        if (send_lateness_s > config.loop_deadline_trip_s) {
          throw std::runtime_error(
              "control loop became stale before a motor exchange");
        }
        if (options.mode == Mode::kShadow) {
          next[index] = bus->readStateBrake(config.motor_ids[index]);
          validateFeedback(next[index], config.motor_ids[index],
                           bus->brakeMode(), config);
        } else {
          qmini_arm::MotorCommand command;
          command.torque_ff_nm = rotor_command[index];
          command.position_rad = states[index].position_rad;
          command.velocity_rad_s = 0.0;
          command.kp = 0.0;
          command.kd = config.kd_rotor;
          next[index] = bus->exchange(config.motor_ids[index], command);
          validateFeedback(next[index], config.motor_ids[index], bus->focMode(),
                           config);
        }
        const double next_joint_velocity =
            config.directions[index] * next[index].velocity_rad_s /
            config.gear_ratio;
        speed_guard.enforceHardTrip(index, next_joint_velocity);
      }
      if (stop_requested) break;
      states = next;
      previous_rotor_command = rotor_command;

      const double computation_s = std::chrono::duration<double>(
                                       Clock::now() - cycle_started)
                                       .count();
      if (computation_s > config.loop_deadline_trip_s) {
        throw std::runtime_error("control loop deadline trip");
      }
      if (cycle % 10 == 0) {
        std::cout << "t=" << std::fixed << std::setprecision(3) << elapsed
                  << " ramp=" << ramp << " saturated=" << saturated << ' ';
        printVector("q", position);
        std::cout << ' ';
        printVector("tau_joint", joint_torque);
        std::cout << ' ';
        printVector("tau_rotor_cmd", rotor_command);
        std::cout << '\n';
        flushOutputOrThrow();
      }
      ++cycle;
      deadline += period;
      const double completion_lateness_s =
          std::chrono::duration<double>(Clock::now() - deadline).count();
      if (completion_lateness_s > config.loop_deadline_trip_s) {
        throw std::runtime_error(
            "control loop completion exceeded the configured watchdog");
      }
      std::this_thread::sleep_until(deadline);
    }
    bool zero_foc_confirmed = false;
    if (options.mode == Mode::kFoc && !stop_requested) {
      // The symmetric envelope has already reduced gravity feed-forward to
      // nearly zero. Confirm an exact zero-torque FOC frame before switching
      // back to BRAKE on a normal bounded exit.
      const auto zero_foc_deadline =
          Clock::now() + std::chrono::duration_cast<Clock::duration>(
                             std::chrono::duration<double>(
                                 config.loop_deadline_trip_s));
      for (int round = 0; round < 3 && !stop_requested; ++round) {
        for (std::size_t index = 0;
             index < states.size() && !stop_requested; ++index) {
          if (Clock::now() > zero_foc_deadline) {
            throw std::runtime_error(
                "zero-torque FOC confirmation exceeded the watchdog");
          }
          qmini_arm::MotorCommand command;
          command.torque_ff_nm = 0.0;
          command.position_rad = states[index].position_rad;
          command.velocity_rad_s = 0.0;
          command.kp = 0.0;
          command.kd = config.kd_rotor;
          states[index] = bus->exchange(config.motor_ids[index], command);
          validateFeedback(states[index], config.motor_ids[index],
                           bus->focMode(), config);
        }
      }
      zero_foc_confirmed = !stop_requested;
    }
    // Do not put potentially blocking logging ahead of the safety command.
    finalBrake(*bus, config);
    if (zero_foc_confirmed) {
      std::cerr << "normal ramp-down complete; zero-torque FOC confirmed\n";
    }
    std::cerr << "timing: minor_deadline_misses=" << minor_deadline_misses
              << " maximum_lateness_ms=" << std::fixed << std::setprecision(3)
              << 1000.0 * std::max(0.0, maximum_lateness_s) << '\n';
    return stop_requested ? 128 + static_cast<int>(stop_signal) : 0;
  } catch (...) {
    if (bus) finalBrake(*bus, config);
    throw;
  }
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parseOptions(argc, argv);
    const ControllerConfig config = loadConfig(options.config_path);
    if (options.mode == Mode::kDryRun) {
      dryRun(config);
      return 0;
    }
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    std::signal(SIGHUP, onSignal);
    std::signal(SIGPIPE, SIG_IGN);
    return runHardware(config, options);
  } catch (const std::exception& error) {
    std::cerr << "gravity-compensation FAILED: " << error.what() << '\n';
    return 1;
  }
}
