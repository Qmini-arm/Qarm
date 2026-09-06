#include <algorithm>
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
#include "qmini_arm/motor_bus.hpp"
#include "qmini_arm/safety.hpp"
#include "qmini_arm/sine_trajectory.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

volatile std::sig_atomic_t g_stop_requested = 0;

void onSignal(int) { g_stop_requested = 1; }

/*
 * 四电机正弦位置测试原理
 * ----------------------
 * Unitree M8010 FOC 控制器内部使用转子侧力位混合控制律：
 *
 *   tau_cmd = tau_ff + kp*(q_des-q) + kd*(dq_des-dq)
 *
 * 本程序令 tau_ff=0、dq_des=0，通过 kp 产生位置回复力矩，kd 提供速度
 * 阻尼。轨迹在减速器后的输出轴侧生成，然后乘以减速比 r≈6.33，叠加
 * 到每台电机各自的启动转子角 q_start：
 *
 *   q_des_rotor[i] = q_start_rotor[i]
 *                  + direction*r*q_sine_output(t)
 *
 * 四台电机共用同一个 t 和相对目标，但 M8010 是请求—应答协议，所以每
 * 个周期内仍需按 ID 顺序轮询，并不使用无反馈的广播控制。
 */

struct Config {
  std::string port = "/dev/ttyUSB0";
  std::vector<int> ids = {0, 1, 2, 3};
  double amplitude_deg = 8.0;
  double center_deg = 0.0;
  double period_s = 4.0;
  double duration_s = 12.0;
  double ramp_s = 2.0;
  double settle_s = 0.5;
  double rate_hz = 100.0;
  double print_hz = 20.0;
  double kp_rotor = 0.2;
  double kd_rotor = 0.03;
  int direction = 1;
  double travel_limit_deg = 15.0;
  double speed_limit_rad_s = 0.5;
  double torque_limit_rotor_nm = 1.0;
  int temperature_limit_c = 60;
  bool assume_yes = false;
  bool dry_run = false;
};

qmini_arm::SineTrajectoryConfig trajectoryConfig(const Config& config) {
  qmini_arm::SineTrajectoryConfig result;
  result.amplitude_rad = config.amplitude_deg * kDegToRad;
  result.center_rad = config.center_deg * kDegToRad;
  result.period_s = config.period_s;
  result.ramp_s = config.ramp_s;
  return result;
}

qmini_arm::SafetyLimits safetyLimits(const Config& config, int foc_mode) {
  qmini_arm::SafetyLimits result;
  result.output_speed_limit_rad_s = config.speed_limit_rad_s;
  result.rotor_torque_limit_nm = config.torque_limit_rotor_nm;
  result.output_travel_limit_rad = config.travel_limit_deg * kDegToRad;
  result.temperature_limit_c = config.temperature_limit_c;
  result.expected_mode = foc_mode;
  return result;
}

void printUsage(const char* program) {
  std::cout
      << "Usage: " << program << " [options]\n\n"
      << "Drive GO-M8010-6 IDs with one relative output-side sine target.\n\n"
      << "Options (defaults shown):\n"
      << "  --port PATH                 /dev/ttyUSB0\n"
      << "  --ids LIST                  0,1,2,3\n"
      << "  --id N                      single-motor compatibility option\n"
      << "  --amplitude-deg DEG         8.0\n"
      << "  --center-deg DEG            0.0\n"
      << "  --period-s SEC              4.0\n"
      << "  --duration-s SEC            12.0\n"
      << "  --ramp-s SEC                2.0\n"
      << "  --settle-s SEC              0.5\n"
      << "  --rate-hz HZ                100.0\n"
      << "  --print-hz HZ               20.0\n"
      << "  --kp-rotor VALUE            0.2\n"
      << "  --kd-rotor VALUE            0.03\n"
      << "  --direction SIGN            +1 or -1 for every listed motor\n"
      << "  --travel-limit-deg DEG      15.0\n"
      << "  --speed-limit-rad-s VALUE   0.5 output side\n"
      << "  --tau-limit-rotor-nm VALUE  1.0\n"
      << "  --temp-limit-c C            60\n"
      << "  --yes                       skip the MOVE confirmation\n"
      << "  --dry-run                   do not open the serial port\n"
      << "  -h, --help                  show this help\n";
}

Config parseArguments(int argc, char** argv) {
  Config config;
  bool single_id_seen = false;
  bool id_list_seen = false;
  for (int i = 1; i < argc; ++i) {
    const std::string option = argv[i];
    if (option == "-h" || option == "--help") {
      printUsage(argv[0]);
      std::exit(0);
    } else if (option == "--yes") {
      config.assume_yes = true;
    } else if (option == "--dry-run") {
      config.dry_run = true;
    } else if (option == "--port") {
      config.port = qmini_arm::cli::requireValue(argc, argv, i, option);
    } else if (option == "--id") {
      if (id_list_seen) {
        throw std::runtime_error("--id and --ids cannot be used together");
      }
      single_id_seen = true;
      config.ids = {qmini_arm::cli::parseInt(
          option, qmini_arm::cli::requireValue(argc, argv, i, option))};
    } else if (option == "--ids") {
      if (single_id_seen) {
        throw std::runtime_error("--id and --ids cannot be used together");
      }
      id_list_seen = true;
      config.ids = qmini_arm::cli::parseIds(
          qmini_arm::cli::requireValue(argc, argv, i, option));
    } else if (option == "--amplitude-deg") {
      config.amplitude_deg = qmini_arm::cli::parseDouble(
          option, qmini_arm::cli::requireValue(argc, argv, i, option));
    } else if (option == "--center-deg") {
      config.center_deg = qmini_arm::cli::parseDouble(
          option, qmini_arm::cli::requireValue(argc, argv, i, option));
    } else if (option == "--period-s") {
      config.period_s = qmini_arm::cli::parseDouble(
          option, qmini_arm::cli::requireValue(argc, argv, i, option));
    } else if (option == "--duration-s") {
      config.duration_s = qmini_arm::cli::parseDouble(
          option, qmini_arm::cli::requireValue(argc, argv, i, option));
    } else if (option == "--ramp-s") {
      config.ramp_s = qmini_arm::cli::parseDouble(
          option, qmini_arm::cli::requireValue(argc, argv, i, option));
    } else if (option == "--settle-s") {
      config.settle_s = qmini_arm::cli::parseDouble(
          option, qmini_arm::cli::requireValue(argc, argv, i, option));
    } else if (option == "--rate-hz") {
      config.rate_hz = qmini_arm::cli::parseDouble(
          option, qmini_arm::cli::requireValue(argc, argv, i, option));
    } else if (option == "--print-hz") {
      config.print_hz = qmini_arm::cli::parseDouble(
          option, qmini_arm::cli::requireValue(argc, argv, i, option));
    } else if (option == "--kp-rotor") {
      config.kp_rotor = qmini_arm::cli::parseDouble(
          option, qmini_arm::cli::requireValue(argc, argv, i, option));
    } else if (option == "--kd-rotor") {
      config.kd_rotor = qmini_arm::cli::parseDouble(
          option, qmini_arm::cli::requireValue(argc, argv, i, option));
    } else if (option == "--direction") {
      config.direction = qmini_arm::cli::parseInt(
          option, qmini_arm::cli::requireValue(argc, argv, i, option));
    } else if (option == "--travel-limit-deg") {
      config.travel_limit_deg = qmini_arm::cli::parseDouble(
          option, qmini_arm::cli::requireValue(argc, argv, i, option));
    } else if (option == "--speed-limit-rad-s") {
      config.speed_limit_rad_s = qmini_arm::cli::parseDouble(
          option, qmini_arm::cli::requireValue(argc, argv, i, option));
    } else if (option == "--tau-limit-rotor-nm") {
      config.torque_limit_rotor_nm = qmini_arm::cli::parseDouble(
          option, qmini_arm::cli::requireValue(argc, argv, i, option));
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
  qmini_arm::cli::validateIds(config.ids);
  if (!(config.amplitude_deg > 0.0 && config.amplitude_deg <= 30.0)) {
    throw std::runtime_error("--amplitude-deg must be in (0, 30]");
  }
  if (std::abs(config.center_deg) > 30.0) {
    throw std::runtime_error("absolute --center-deg must be <= 30");
  }
  if (!(config.travel_limit_deg > 0.0 &&
        config.travel_limit_deg <= 30.0)) {
    throw std::runtime_error("--travel-limit-deg must be in (0, 30]");
  }
  if (std::abs(config.center_deg) + config.amplitude_deg >
      config.travel_limit_deg) {
    throw std::runtime_error(
        "planned |center| + amplitude exceeds --travel-limit-deg");
  }
  if (!(config.period_s >= 0.5 && config.period_s <= 60.0)) {
    throw std::runtime_error("--period-s must be in [0.5, 60]");
  }
  if (!(config.duration_s > 0.0 && config.duration_s <= 60.0)) {
    throw std::runtime_error("--duration-s must be in (0, 60]");
  }
  if (!(config.ramp_s > 0.0 &&
        config.ramp_s <= config.duration_s / 2.0)) {
    throw std::runtime_error("--ramp-s must be in (0, duration/2]");
  }
  if (!(config.settle_s >= 0.0 && config.settle_s <= 5.0)) {
    throw std::runtime_error("--settle-s must be in [0, 5]");
  }
  if (!(config.rate_hz >= 10.0 && config.rate_hz <= 200.0)) {
    throw std::runtime_error("--rate-hz must be in [10, 200]");
  }
  if (!(config.print_hz > 0.0 && config.print_hz <= config.rate_hz)) {
    throw std::runtime_error("--print-hz must be in (0, rate-hz]");
  }
  if (!(config.kp_rotor > 0.0 && config.kp_rotor <= 0.5)) {
    throw std::runtime_error("--kp-rotor must be in (0, 0.5]");
  }
  if (!(config.kd_rotor >= 0.0 && config.kd_rotor <= 0.2)) {
    throw std::runtime_error("--kd-rotor must be in [0, 0.2]");
  }
  if (config.direction != -1 && config.direction != 1) {
    throw std::runtime_error("--direction must be +1 or -1");
  }
  if (!(config.speed_limit_rad_s > 0.0 &&
        config.speed_limit_rad_s <= 2.0)) {
    throw std::runtime_error("--speed-limit-rad-s must be in (0, 2]");
  }
  if (!(config.torque_limit_rotor_nm > 0.0 &&
        config.torque_limit_rotor_nm <= 5.0)) {
    throw std::runtime_error("--tau-limit-rotor-nm must be in (0, 5]");
  }
  if (config.temperature_limit_c < 30 ||
      config.temperature_limit_c > 85) {
    throw std::runtime_error("--temp-limit-c must be in [30, 85]");
  }
  if (qmini_arm::conservativePeakSpeedRadS(trajectoryConfig(config)) >
      config.speed_limit_rad_s) {
    throw std::runtime_error(
        "planned trajectory can exceed the speed limit during ramp");
  }
}

void printConfig(const Config& config, double gear_ratio) {
  std::cout << std::fixed << std::setprecision(3)
            << "Motors: GO-M8010-6, port=" << config.port
            << ", ids=" << qmini_arm::cli::formatIds(config.ids)
            << ", gear_ratio=" << gear_ratio << '\n'
            << "Trajectory: amplitude=" << config.amplitude_deg
            << " deg, center=" << config.center_deg
            << " deg, period=" << config.period_s
            << " s, duration=" << config.duration_s
            << " s, ramp=" << config.ramp_s
            << " s, direction=" << config.direction << '\n'
            << "Rotor gains: kp=" << config.kp_rotor
            << ", kd=" << config.kd_rotor
            << "; output speed guard=" << config.speed_limit_rad_s
            << " rad/s\n";
}

void runDry(const Config& config) {
  const qmini_arm::SineTrajectoryConfig trajectory =
      trajectoryConfig(config);
  std::cout << "DRY RUN: serial port will not be opened.\n"
            << "time_s,target_output_deg,target_rotor_offset_rad\n";
  constexpr int kSamples = 20;
  constexpr double kM8010Ratio = 6.33;
  for (int i = 0; i <= kSamples; ++i) {
    const double time_s = config.duration_s * i / kSamples;
    const double offset =
        qmini_arm::sinePositionOffsetRad(trajectory, time_s);
    std::cout << std::fixed << std::setprecision(5) << time_s << ','
              << offset * kRadToDeg << ','
              << config.direction * kM8010Ratio * offset << '\n';
  }
}

std::vector<double> acquireStartPositions(qmini_arm::MotorBus& bus,
                                          const Config& config) {
  std::vector<double> positions(config.ids.size(), 0.0);
  for (int round = 0; round < 3; ++round) {
    for (std::size_t i = 0; i < config.ids.size(); ++i) {
      const qmini_arm::MotorState state =
          bus.readStateZeroOutput(config.ids[i]);
      qmini_arm::validateBasicState(state, bus.focMode(),
                                    config.temperature_limit_c);
      positions[i] = state.position_rad;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return positions;
}

void settleAtStart(qmini_arm::MotorBus& bus, const Config& config,
                   const std::vector<double>& start_positions,
                   const qmini_arm::SafetyLimits& limits) {
  if (config.settle_s <= 0.0 || g_stop_requested) return;

  using Clock = std::chrono::steady_clock;
  const auto period = std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(1.0 / config.rate_hz));
  const auto started = Clock::now();
  auto deadline = started;
  while (!g_stop_requested) {
    if (std::chrono::duration<double>(Clock::now() - started).count() >=
        config.settle_s) {
      break;
    }
    for (std::size_t i = 0; i < config.ids.size(); ++i) {
      qmini_arm::MotorCommand command;
      command.position_rad = start_positions[i];
      command.kp = config.kp_rotor;
      command.kd = config.kd_rotor;
      const qmini_arm::MotorState state =
          bus.exchange(config.ids[i], command);
      qmini_arm::enforceMotionSafety(state, bus.gearRatio(),
                                     start_positions[i], limits);
    }
    deadline += period;
    std::this_thread::sleep_until(deadline);
  }
}

void runMotor(const Config& config) {
  using Clock = std::chrono::steady_clock;
  std::unique_ptr<qmini_arm::MotorBus> bus;
  bool completed_normally = false;
  std::size_t cycles = 0;
  std::size_t overruns = 0;

  try {
    bus.reset(new qmini_arm::MotorBus(config.port));
    if (std::abs(bus->gearRatio() - 6.33) > 0.05) {
      throw std::runtime_error("unexpected M8010 gear ratio: " +
                               std::to_string(bus->gearRatio()));
    }
    const qmini_arm::SafetyLimits limits =
        safetyLimits(config, bus->focMode());
    const std::vector<double> start_positions =
        acquireStartPositions(*bus, config);

    std::cout << std::fixed << std::setprecision(5);
    for (std::size_t i = 0; i < config.ids.size(); ++i) {
      std::cout << "Valid initial feedback. id=" << config.ids[i]
                << ", start_q_rotor=" << start_positions[i]
                << " rad; this is not an absolute joint zero.\n";
    }
    std::cout << "time_s,motor_id,target_deg,actual_deg,error_deg,"
                 "velocity_rad_s,tau_rotor_est_nm,temp_c,exchange_ms\n";

    const qmini_arm::SineTrajectoryConfig trajectory =
        trajectoryConfig(config);
    const auto loop_period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / config.rate_hz));
    const double gap_limit_s = std::max(0.1, 3.0 / config.rate_hz);
    const auto started = Clock::now();
    auto deadline = started;
    auto previous_cycle = started;
    auto next_print = started;

    while (!g_stop_requested) {
      const auto cycle_start = Clock::now();
      const double elapsed_s =
          std::chrono::duration<double>(cycle_start - started).count();
      if (elapsed_s >= config.duration_s) {
        completed_normally = true;
        break;
      }
      if (cycles > 0) {
        const double gap_s =
            std::chrono::duration<double>(cycle_start - previous_cycle).count();
        if (gap_s > gap_limit_s) {
          throw std::runtime_error("control scheduling gap exceeded " +
                                   std::to_string(gap_limit_s * 1000.0) +
                                   " ms");
        }
      }
      previous_cycle = cycle_start;

      const double output_offset =
          qmini_arm::sinePositionOffsetRad(trajectory, elapsed_s);
      const double gain_envelope =
          qmini_arm::smoothStep01(elapsed_s / config.ramp_s);
      std::vector<qmini_arm::MotorState> states(config.ids.size());
      std::vector<double> exchange_times_ms(config.ids.size(), 0.0);

      for (std::size_t i = 0; i < config.ids.size(); ++i) {
        qmini_arm::MotorCommand command;
        command.position_rad =
            start_positions[i] + config.direction * bus->gearRatio() *
                                     output_offset;
        command.kp = config.kp_rotor * gain_envelope;
        command.kd = config.kd_rotor * gain_envelope;

        const auto exchange_started = Clock::now();
        states[i] = bus->exchange(config.ids[i], command);
        exchange_times_ms[i] =
            std::chrono::duration<double, std::milli>(Clock::now() -
                                                      exchange_started)
                .count();
        qmini_arm::enforceMotionSafety(states[i], bus->gearRatio(),
                                       start_positions[i], limits);
      }

      if (cycle_start >= next_print) {
        const double target_deg = output_offset * kRadToDeg;
        for (std::size_t i = 0; i < config.ids.size(); ++i) {
          const double actual_rad =
              config.direction *
              (states[i].position_rad - start_positions[i]) /
              bus->gearRatio();
          const double actual_deg = actual_rad * kRadToDeg;
          const double velocity = config.direction * states[i].velocity_rad_s /
                                  bus->gearRatio();
          std::cout << std::fixed << std::setprecision(4) << elapsed_s << ','
                    << config.ids[i] << ',' << target_deg << ',' << actual_deg
                    << ',' << target_deg - actual_deg << ',' << velocity << ','
                    << states[i].torque_estimate_nm << ','
                    << states[i].temperature_c << ',' << exchange_times_ms[i]
                    << '\n';
        }
        next_print = cycle_start + std::chrono::duration_cast<Clock::duration>(
                                       std::chrono::duration<double>(
                                           1.0 / config.print_hz));
      }

      ++cycles;
      deadline += loop_period;
      const auto now = Clock::now();
      if (now < deadline) {
        std::this_thread::sleep_until(deadline);
      } else {
        ++overruns;
        deadline = now;
      }
    }

    if (completed_normally) {
      settleAtStart(*bus, config, start_positions, limits);
    }
  } catch (...) {
    if (bus) {
      const int acknowledged = bus->sendZeroOutput(config.ids);
      std::cerr << "EXIT zero-output acknowledgements: " << acknowledged << '/'
                << 3 * config.ids.size() << '\n';
    }
    throw;
  }

  if (bus) {
    const int acknowledged = bus->sendZeroOutput(config.ids);
    const int expected = static_cast<int>(3 * config.ids.size());
    std::cout << "EXIT zero-output acknowledgements: " << acknowledged << '/'
              << expected << '\n';
    if (acknowledged != expected) {
      std::cerr << "WARNING: final zero-output was not fully confirmed; cut "
                   "motor power.\n";
    }
  }
  std::cout << "STATS cycles=" << cycles << ", overruns=" << overruns
            << ", motor_exchanges=" << cycles * config.ids.size()
            << ", completed=" << (completed_normally ? "yes" : "interrupted")
            << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Config config = parseArguments(argc, argv);
    validateConfig(config);
    printConfig(config, 6.33);

    if (config.dry_run) {
      runDry(config);
      return 0;
    }

    std::cout << "WARNING: This actively moves " << config.ids.size()
              << " secured, unloaded motors (IDs "
              << qmini_arm::cli::formatIds(config.ids) << ").\n"
              << "All listed motors receive the same relative direction and "
                 "trajectory. This is not an assembled-arm controller.\n"
              << "Type MOVE only with a physical power cutoff ready.\n";
    if (!config.assume_yes) {
      std::cout << "Type MOVE to open the serial port and start: " << std::flush;
      std::string confirmation;
      std::getline(std::cin, confirmation);
      if (confirmation != "MOVE") {
        std::cout << "Cancelled.\n";
        return 2;
      }
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    runMotor(config);
    return g_stop_requested ? 130 : 0;
  } catch (const std::exception& error) {
    std::cerr << "FAILED: " << error.what() << '\n';
    return 1;
  }
}
