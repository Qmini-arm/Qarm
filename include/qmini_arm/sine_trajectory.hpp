#ifndef QMINI_ARM_SINE_TRAJECTORY_HPP_
#define QMINI_ARM_SINE_TRAJECTORY_HPP_

namespace qmini_arm {

// All trajectory values use SI units and describe the reducer output / joint
// side. CLI applications may present degrees, but conversions happen at edges.
struct SineTrajectoryConfig {
  double amplitude_rad = 0.0;
  double center_rad = 0.0;
  double period_s = 4.0;
  double ramp_s = 2.0;
};

double smoothStep01(double value);
double sinePositionOffsetRad(const SineTrajectoryConfig& config,
                             double elapsed_s);

// Conservative sum of the sine and startup-envelope peak-speed bounds.
double conservativePeakSpeedRadS(const SineTrajectoryConfig& config);

}  // namespace qmini_arm

#endif  // QMINI_ARM_SINE_TRAJECTORY_HPP_

