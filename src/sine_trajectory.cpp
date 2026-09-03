#include "qmini_arm/sine_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace qmini_arm {

namespace {

constexpr double kPi = 3.14159265358979323846;

void validate(const SineTrajectoryConfig& config) {
  if (!std::isfinite(config.amplitude_rad) ||
      !std::isfinite(config.center_rad) ||
      !std::isfinite(config.period_s) || !std::isfinite(config.ramp_s)) {
    throw std::invalid_argument("sine trajectory values must be finite");
  }
  if (config.amplitude_rad < 0.0) {
    throw std::invalid_argument("sine amplitude must be non-negative");
  }
  if (config.period_s <= 0.0 || config.ramp_s <= 0.0) {
    throw std::invalid_argument("sine period and ramp must be positive");
  }
}

}  // namespace

double smoothStep01(double value) {
  const double x = std::max(0.0, std::min(1.0, value));
  return x * x * (3.0 - 2.0 * x);
}

double sinePositionOffsetRad(const SineTrajectoryConfig& config,
                             double elapsed_s) {
  validate(config);
  if (!std::isfinite(elapsed_s)) {
    throw std::invalid_argument("trajectory time must be finite");
  }
  const double envelope = smoothStep01(elapsed_s / config.ramp_s);
  return envelope *
         (config.center_rad +
          config.amplitude_rad *
              std::sin(2.0 * kPi * elapsed_s / config.period_s));
}

double conservativePeakSpeedRadS(const SineTrajectoryConfig& config) {
  validate(config);
  const double sine_peak =
      config.amplitude_rad * 2.0 * kPi / config.period_s;
  const double ramp_peak =
      1.5 * (std::abs(config.center_rad) + config.amplitude_rad) /
      config.ramp_s;
  return sine_peak + ramp_peak;
}

}  // namespace qmini_arm

