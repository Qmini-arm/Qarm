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

#include "qmini_arm/gravity_control.hpp"
#include "qmini_arm/gravity_model.hpp"
#include "qmini_arm/joint_trajectory.hpp"
#include "qmini_arm/motor_bus.hpp"
#include "qmini_arm/sine_trajectory.hpp"
#include "qmini_arm/types.hpp"

namespace {

using Clock = std::chrono::steady_clock;
using qmini_arm::JointVector;

constexpr double kKpRotor = 0.20;
constexpr double kKdRotor = 0.03;
constexpr double kMaximumPlanVelocityRadS = 0.30;
constexpr double kMaximumPlanAccelerationRadS2 = 0.60;
constexpr double kMaximumSamplePeriodS = 0.05;
constexpr double kMaximumPlanDurationS = 120.0;
constexpr double kPlanFinalToleranceRad = 1e-4;
constexpr double kMeasuredStartToleranceRad = 0.03;
constexpr double kTrackingErrorTripRad = 0.20;
constexpr double kFinalPositionToleranceRad = 0.04;
constexpr double kGainRampS = 0.01;

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
  double result = 0.0;
  try {
    result = std::stod(value, &used);
  } catch (const std::exception&) {
    throw std::runtime_error(name + " must be a number");
  }
  if (used != value.size() || !std::isfinite(result)) {
    throw std::runtime_error(name + " must be finite");
  }
  return result;
}

int parseInt(const std::string& name, const std::string& value) {
  std::size_t used = 0;
  int result = 0;
  try {
    result = std::stoi(value, &used);
  } catch (const std::exception&) {
    throw std::runtime_error(name + " must be an integer");
  }
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

std::map<std::string, std::string> readKeyValues(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot open config: " + path);
  std::map<std::string, std::string> result;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(stream, line)) {
    ++line_number;
    const std::size_t comment = line.find('#');
    if (comment != std::string::npos) line.erase(comment);
    line = trim(line);
    if (line.empty()) continue;
    const std::size_t equals = line.find('=');
    if (equals == std::string::npos) {
      throw std::runtime_error("config line " +
                               std::to_string(line_number) + " has no '='");
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

struct HomeConfig {
  std::string device;
  std::string expected_board_boot_id;
  double gear_ratio = 0.0;
  std::array<int, 6> motor_ids{};
  std::array<int, 6> directions{};
  JointVector rotor_at_reference_rad{};
  JointVector reference_joint_rad{};
  JointVector soft_lower_rad{};
  JointVector soft_upper_rad{};
  double startup_limit_margin_rad = 0.0;
  double runtime_limit_margin_rad = 0.0;
  JointVector rotor_torque_caps_nm{};
  double rotor_torque_slew_nm_per_cycle = 0.0;
  JointVector joint_speed_trip_rad_s{};
  JointVector joint_speed_hard_trip_rad_s{};
  int joint_speed_trip_consecutive_cycles = 0;
  double rotor_feedback_torque_trip_nm = 0.0;
  int temperature_trip_c = 0;
  double loop_deadline_trip_s = 0.0;
};

HomeConfig loadConfig(const std::string& path) {
  const auto values = readKeyValues(path);
  if (parseInt("schema_version", required(values, "schema_version")) != 2) {
    throw std::runtime_error("return-to-zero requires gravity config schema 2");
  }
  HomeConfig config;
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
      "reference_joint_rad", required(values, "reference_joint_rad"),
      parseDouble);
  config.soft_lower_rad = parseArray<double>(
      "soft_lower_rad", required(values, "soft_lower_rad"), parseDouble);
  config.soft_upper_rad = parseArray<double>(
      "soft_upper_rad", required(values, "soft_upper_rad"), parseDouble);
  config.startup_limit_margin_rad = parseDouble(
      "startup_limit_margin_rad", required(values, "startup_limit_margin_rad"));
  config.runtime_limit_margin_rad = parseDouble(
      "runtime_limit_margin_rad", required(values, "runtime_limit_margin_rad"));
  config.rotor_torque_caps_nm = parseArray<double>(
      "rotor_torque_caps_nm", required(values, "rotor_torque_caps_nm"),
      parseDouble);
  config.rotor_torque_slew_nm_per_cycle = parseDouble(
      "rotor_torque_slew_nm_per_cycle",
      required(values, "rotor_torque_slew_nm_per_cycle"));
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
  config.loop_deadline_trip_s = parseDouble(
      "loop_deadline_trip_s", required(values, "loop_deadline_trip_s"));

  if (config.device.empty() || config.expected_board_boot_id.empty() ||
      config.gear_ratio <= 0.0 || config.startup_limit_margin_rad <= 0.0 ||
      config.runtime_limit_margin_rad <= 0.0 ||
      config.startup_limit_margin_rad < config.runtime_limit_margin_rad ||
      config.rotor_torque_slew_nm_per_cycle <= 0.0 ||
      config.rotor_torque_slew_nm_per_cycle > 0.05 ||
      config.joint_speed_trip_consecutive_cycles < 2 ||
      config.joint_speed_trip_consecutive_cycles > 5 ||
      config.rotor_feedback_torque_trip_nm <= 0.0 ||
      config.rotor_feedback_torque_trip_nm > 3.0 ||
      config.temperature_trip_c < 30 || config.temperature_trip_c > 60 ||
      config.loop_deadline_trip_s < 0.02 ||
      config.loop_deadline_trip_s > 0.1) {
    throw std::runtime_error("return-to-zero config violates safety bounds");
  }
  for (std::size_t index = 0; index < 6; ++index) {
    if (config.motor_ids[index] != static_cast<int>(index) ||
        (config.directions[index] != -1 && config.directions[index] != 1) ||
        config.soft_lower_rad[index] >= config.soft_upper_rad[index] ||
        config.rotor_torque_caps_nm[index] <= 0.0 ||
        config.rotor_torque_caps_nm[index] > 2.0 ||
        config.joint_speed_trip_rad_s[index] <= 0.0 ||
        config.joint_speed_hard_trip_rad_s[index] <=
            config.joint_speed_trip_rad_s[index] ||
        config.joint_speed_hard_trip_rad_s[index] > 2.0) {
      throw std::runtime_error("invalid per-joint return-to-zero config");
    }
  }
  return config;
}

enum class Mode { kDryRun, kFoc };

struct Options {
  std::string config_path = "config/gravity_comp.conf";
  std::string trajectory_path;
  Mode mode = Mode::kDryRun;
  bool mode_was_set = false;
  bool acknowledge_supported_arm = false;
  bool acknowledge_estop_ready = false;
  bool confirm_same_motor_power_cycle = false;
  bool confirm_collision_checked_plan = false;
};

void printUsage(const char* program) {
  std::cout
      << "Usage: " << program
      << " --trajectory FILE (--dry-run | --enable-foc) [options]\n\n"
      << "The CSV must come from qarm-sim plan-home and finish at URDF zero.\n\n"
      << "Options:\n"
      << "  --config FILE                     gravity config schema 2\n"
      << "  --trajectory FILE                 collision-checked joint CSV\n"
      << "  --dry-run                         validate without serial access\n"
      << "  --enable-foc                      execute the bounded trajectory\n"
      << "  --acknowledge-supported-arm\n"
      << "  --acknowledge-estop-ready\n"
      << "  --confirm-same-motor-power-cycle\n"
      << "  --confirm-collision-checked-plan\n";
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
    } else if (option == "--trajectory") {
      result.trajectory_path = value();
    } else if (option == "--dry-run") {
      set_mode(Mode::kDryRun);
    } else if (option == "--enable-foc") {
      set_mode(Mode::kFoc);
    } else if (option == "--acknowledge-supported-arm") {
      result.acknowledge_supported_arm = true;
    } else if (option == "--acknowledge-estop-ready") {
      result.acknowledge_estop_ready = true;
    } else if (option == "--confirm-same-motor-power-cycle") {
      result.confirm_same_motor_power_cycle = true;
    } else if (option == "--confirm-collision-checked-plan") {
      result.confirm_collision_checked_plan = true;
    } else {
      throw std::runtime_error("unknown option: " + option);
    }
  }
  if (!result.mode_was_set) {
    throw std::runtime_error("choose --dry-run or --enable-foc");
  }
  if (result.trajectory_path.empty()) {
    throw std::runtime_error("--trajectory is required");
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

JointVector jointPosition(const std::array<qmini_arm::MotorState, 6>& states,
                          const HomeConfig& config) {
  JointVector result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    result[index] = config.reference_joint_rad[index] +
                    config.directions[index] *
                        (states[index].position_rad -
                         config.rotor_at_reference_rad[index]) /
                        config.gear_ratio;
  }
  return result;
}

JointVector jointVelocity(const std::array<qmini_arm::MotorState, 6>& states,
                          const HomeConfig& config) {
  JointVector result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    result[index] = config.directions[index] * states[index].velocity_rad_s /
                    config.gear_ratio;
  }
  return result;
}

double rotorPosition(double joint_position_rad,
                     std::size_t index,
                     const HomeConfig& config) {
  return config.rotor_at_reference_rad[index] +
         config.directions[index] * config.gear_ratio *
             (joint_position_rad - config.reference_joint_rad[index]);
}

void validateFeedback(const qmini_arm::MotorState& state,
                      int expected_id,
                      int expected_mode,
                      const HomeConfig& config) {
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
                        const HomeConfig& config,
                        double margin) {
  for (std::size_t index = 0; index < position.size(); ++index) {
    if (position[index] <= config.soft_lower_rad[index] + margin ||
        position[index] >= config.soft_upper_rad[index] - margin) {
      throw std::runtime_error("joint_" + std::to_string(index + 1) +
                               " lacks the required soft-limit margin");
    }
  }
}

std::array<qmini_arm::MotorState, 6> readBrake(
    qmini_arm::MotorBus& bus, const HomeConfig& config) {
  std::array<qmini_arm::MotorState, 6> result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    result[index] = bus.readStateBrake(config.motor_ids[index]);
    validateFeedback(result[index], config.motor_ids[index], bus.brakeMode(),
                     config);
  }
  return result;
}

void finalBrake(qmini_arm::MotorBus& bus, const HomeConfig& config) {
  const std::vector<int> ids(config.motor_ids.begin(), config.motor_ids.end());
  const int acknowledgements = bus.sendBrake(ids, 3);
  std::cerr << "BRAKE acknowledgements=" << acknowledgements << "/18\n";
  if (acknowledgements != 18) {
    std::cerr << "WARNING: BRAKE was not confirmed by every motor; use the "
                 "physical power cutoff.\n";
  }
}

void printVector(const char* name, const JointVector& values) {
  std::cout << name << "=[";
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index) std::cout << ',';
    std::cout << std::fixed << std::setprecision(6) << values[index];
  }
  std::cout << ']';
}

void validateTrajectory(const qmini_arm::JointTrajectory& trajectory,
                        const HomeConfig& config) {
  qmini_arm::validateHomeTrajectory(
      trajectory, config.soft_lower_rad, config.soft_upper_rad,
      config.runtime_limit_margin_rad, kMaximumPlanVelocityRadS,
      kMaximumPlanAccelerationRadS2, kMaximumSamplePeriodS,
      kMaximumPlanDurationS, kPlanFinalToleranceRad);
}

void dryRun(const qmini_arm::JointTrajectory& trajectory) {
  std::cout << "dry-run: no serial access\n"
            << "samples=" << trajectory.size()
            << " duration_s=" << std::fixed << std::setprecision(3)
            << trajectory.back().time_s << ' ';
  printVector("start_q", trajectory.front().position_rad);
  std::cout << ' ';
  printVector("goal_q", trajectory.back().position_rad);
  std::cout << '\n';
}

int runHardware(const HomeConfig& config,
                const qmini_arm::JointTrajectory& trajectory,
                const Options& options) {
  if (!options.acknowledge_supported_arm ||
      !options.acknowledge_estop_ready ||
      !options.confirm_same_motor_power_cycle ||
      !options.confirm_collision_checked_plan) {
    throw std::runtime_error(
        "FOC requires all four support, e-stop, power-cycle, and plan "
        "confirmations");
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

    qmini_arm::PerJointSpeedGuard speed_guard(
        config.joint_speed_trip_rad_s,
        config.joint_speed_hard_trip_rad_s,
        static_cast<std::size_t>(config.joint_speed_trip_consecutive_cycles));
    std::array<qmini_arm::MotorState, 6> states{};
    JointVector actual_position{};
    JointVector actual_velocity{};
    for (int sample = 0; sample < 5; ++sample) {
      states = readBrake(*bus, config);
      actual_position = jointPosition(states, config);
      actual_velocity = jointVelocity(states, config);
      validatePoseLimits(actual_position, config,
                         config.startup_limit_margin_rad);
      speed_guard.observeFrame(actual_velocity);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    speed_guard.reset();
    for (std::size_t index = 0; index < 6; ++index) {
      if (std::abs(actual_position[index] -
                   trajectory.front().position_rad[index]) >
          kMeasuredStartToleranceRad) {
        throw std::runtime_error(
            "measured joint_" + std::to_string(index + 1) +
            " does not match the collision-checked plan start");
      }
    }

    std::cout << "mode=FOC trajectory=" << options.trajectory_path
              << " samples=" << trajectory.size()
              << " duration_s=" << std::fixed << std::setprecision(3)
              << trajectory.back().time_s << '\n';
    printVector("measured_start_q", actual_position);
    std::cout << '\n';
    flushOutputOrThrow();

    qmini_arm::GravityModel gravity;
    // Seed the slew state from the measured BRAKE pose. This prevents the
    // arm from briefly dropping while a 100% gravity feed-forward command is
    // ramped up from zero; all subsequent changes remain slew-limited.
    JointVector previous_rotor_torque =
        qmini_arm::jointGravityToRotorTorque(
            gravity.compensationTorque(actual_position), 1.0,
            config.directions, config.gear_ratio);
    for (std::size_t index = 0; index < previous_rotor_torque.size(); ++index) {
      previous_rotor_torque[index] = std::max(
          -config.rotor_torque_caps_nm[index],
          std::min(config.rotor_torque_caps_nm[index],
                   previous_rotor_torque[index]));
    }
    const auto started = Clock::now();
    std::size_t cycle = 0;
    std::size_t minor_deadline_misses = 0;
    double maximum_lateness_s = 0.0;

    for (const auto& target : trajectory) {
      if (stop_requested) break;
      const auto deadline =
          started + std::chrono::duration_cast<Clock::duration>(
                        std::chrono::duration<double>(target.time_s));
      std::this_thread::sleep_until(deadline);
      const auto cycle_started = Clock::now();
      const double lateness_s =
          std::chrono::duration<double>(cycle_started - deadline).count();
      maximum_lateness_s = std::max(maximum_lateness_s, lateness_s);
      if (lateness_s > config.loop_deadline_trip_s) {
        throw std::runtime_error("return-to-zero control deadline trip");
      }
      if (lateness_s > 0.02) ++minor_deadline_misses;

      actual_position = jointPosition(states, config);
      actual_velocity = jointVelocity(states, config);
      validatePoseLimits(actual_position, config,
                         config.runtime_limit_margin_rad);
      speed_guard.observeFrame(actual_velocity);
      const JointVector joint_torque =
          gravity.compensationTorque(actual_position);
      const JointVector requested_rotor_torque =
          qmini_arm::jointGravityToRotorTorque(
              joint_torque, 1.0, config.directions, config.gear_ratio);
      bool saturated = false;
      const JointVector rotor_torque = qmini_arm::limitRotorTorque(
          requested_rotor_torque, previous_rotor_torque,
          config.rotor_torque_caps_nm,
          config.rotor_torque_slew_nm_per_cycle, &saturated);
      const double gain_envelope =
          qmini_arm::smoothStep01(target.time_s / kGainRampS);

      std::array<qmini_arm::MotorState, 6> next{};
      for (std::size_t index = 0; index < 6; ++index) {
        if (stop_requested) break;
        if (std::chrono::duration<double>(Clock::now() - deadline).count() >
            config.loop_deadline_trip_s) {
          throw std::runtime_error(
              "return-to-zero frame became stale before motor exchange");
        }
        qmini_arm::MotorCommand command;
        command.torque_ff_nm = rotor_torque[index];
        command.position_rad =
            rotorPosition(target.position_rad[index], index, config);
        command.velocity_rad_s = config.directions[index] *
                                 config.gear_ratio *
                                 target.velocity_rad_s[index];
        command.kp = kKpRotor * gain_envelope;
        command.kd = kKdRotor * gain_envelope;
        next[index] = bus->exchange(config.motor_ids[index], command);
        validateFeedback(next[index], config.motor_ids[index], bus->focMode(),
                         config);
        speed_guard.enforceHardTrip(
            index, config.directions[index] * next[index].velocity_rad_s /
                       config.gear_ratio);
      }
      if (stop_requested) break;
      states = next;
      previous_rotor_torque = rotor_torque;
      const JointVector measured = jointPosition(states, config);
      for (std::size_t index = 0; index < 6; ++index) {
        if (std::abs(measured[index] - target.position_rad[index]) >
            kTrackingErrorTripRad) {
          throw std::runtime_error("joint_" + std::to_string(index + 1) +
                                   " exceeded the tracking-error trip");
        }
      }

      if (cycle % 10 == 0) {
        std::cout << "t=" << std::fixed << std::setprecision(3)
                  << target.time_s << " saturated=" << saturated << ' ';
        printVector("q_target", target.position_rad);
        std::cout << ' ';
        printVector("q_actual", measured);
        std::cout << ' ';
        printVector("tau_rotor", rotor_torque);
        std::cout << '\n';
        flushOutputOrThrow();
      }
      ++cycle;
    }

    bool reached_zero = !stop_requested;
    if (reached_zero) {
      actual_position = jointPosition(states, config);
      for (std::size_t index = 0; index < 6; ++index) {
        reached_zero = reached_zero &&
                       std::abs(actual_position[index]) <=
                           kFinalPositionToleranceRad;
      }
    }

    if (!stop_requested && !reached_zero) {
      throw std::runtime_error(
          "trajectory ended but measured joints did not settle at URDF zero");
    }
    // Never put status logging ahead of the BRAKE request.
    finalBrake(*bus, config);
    if (reached_zero) {
      std::cerr << "return-to-zero reached the measured final tolerance\n";
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
    const HomeConfig config = loadConfig(options.config_path);
    const qmini_arm::JointTrajectory trajectory =
        qmini_arm::loadJointTrajectoryCsv(options.trajectory_path);
    validateTrajectory(trajectory, config);
    if (options.mode == Mode::kDryRun) {
      dryRun(trajectory);
      return 0;
    }
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    std::signal(SIGHUP, onSignal);
    std::signal(SIGPIPE, SIG_IGN);
    return runHardware(config, trajectory, options);
  } catch (const std::exception& error) {
    std::cerr << "return-to-zero FAILED: " << error.what() << '\n';
    return 1;
  }
}
