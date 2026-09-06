#include "qmini_arm/joint_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace qmini_arm {

namespace {

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> result;
  std::string normalized = line;
  if (!normalized.empty() && normalized.back() == '\r') normalized.pop_back();
  std::stringstream stream(normalized);
  std::string item;
  while (std::getline(stream, item, ',')) {
    result.push_back(item);
  }
  if (!normalized.empty() && normalized.back() == ',') result.emplace_back();
  return result;
}

double parseNumber(const std::string& text,
                   std::size_t line_number,
                   const std::string& column) {
  std::size_t used = 0;
  double value = 0.0;
  try {
    value = std::stod(text, &used);
  } catch (const std::exception&) {
    throw std::runtime_error("trajectory line " +
                             std::to_string(line_number) + ", " + column +
                             " is not a number");
  }
  if (used != text.size() || !std::isfinite(value)) {
    throw std::runtime_error("trajectory line " +
                             std::to_string(line_number) + ", " + column +
                             " must be finite");
  }
  return value;
}

std::vector<std::string> expectedHeader() {
  std::vector<std::string> result = {"time_s"};
  for (std::size_t joint = 1; joint <= kJointCount; ++joint) {
    result.push_back("joint_" + std::to_string(joint) + "_position_rad");
  }
  for (std::size_t joint = 1; joint <= kJointCount; ++joint) {
    result.push_back("joint_" + std::to_string(joint) + "_velocity_rad_s");
  }
  return result;
}

void validatePositiveFinite(double value, const char* name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be positive");
  }
}

}  // namespace

JointTrajectory parseJointTrajectoryCsv(std::istream& stream) {
  std::string line;
  if (!std::getline(stream, line)) {
    throw std::runtime_error("trajectory CSV is empty");
  }
  if (splitCsv(line) != expectedHeader()) {
    throw std::runtime_error(
        "trajectory CSV header does not match the four-joint home plan");
  }

  JointTrajectory result;
  std::size_t line_number = 1;
  while (std::getline(stream, line)) {
    ++line_number;
    if (line.empty() || line == "\r") continue;
    const std::vector<std::string> columns = splitCsv(line);
    constexpr std::size_t kColumnCount = 1 + 2 * kJointCount;
    if (columns.size() != kColumnCount) {
      throw std::runtime_error("trajectory line " +
                               std::to_string(line_number) +
                               " must contain exactly " +
                               std::to_string(kColumnCount) + " columns");
    }
    JointTrajectorySample sample;
    sample.time_s = parseNumber(columns[0], line_number, "time_s");
    for (std::size_t index = 0; index < qmini_arm::kJointCount; ++index) {
      sample.position_rad[index] = parseNumber(
          columns[1 + index], line_number,
          "joint_" + std::to_string(index + 1) + "_position_rad");
      sample.velocity_rad_s[index] = parseNumber(
          columns[1 + kJointCount + index], line_number,
          "joint_" + std::to_string(index + 1) + "_velocity_rad_s");
    }
    result.push_back(sample);
  }
  return result;
}

JointTrajectory loadJointTrajectoryCsv(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot open trajectory: " + path);
  return parseJointTrajectoryCsv(stream);
}

void validateHomeTrajectory(const JointTrajectory& trajectory,
                            const JointVector& soft_lower_rad,
                            const JointVector& soft_upper_rad,
                            const JointVector& hard_lower_rad,
                            const JointVector& hard_upper_rad,
                            const JointVector& goal_position_rad,
                            bool allow_hard_limit_goal,
                            double limit_margin_rad,
                            double maximum_velocity_rad_s,
                            double maximum_acceleration_rad_s2,
                            double maximum_sample_period_s,
                            double maximum_duration_s,
                            double final_tolerance_rad) {
  if (trajectory.size() < 2) {
    throw std::runtime_error("home trajectory requires at least two samples");
  }
  if (!std::isfinite(limit_margin_rad) || limit_margin_rad < 0.0) {
    throw std::invalid_argument("trajectory limit margin must be non-negative");
  }
  validatePositiveFinite(maximum_velocity_rad_s, "maximum velocity");
  validatePositiveFinite(maximum_acceleration_rad_s2,
                         "maximum acceleration");
  validatePositiveFinite(maximum_sample_period_s, "maximum sample period");
  validatePositiveFinite(maximum_duration_s, "maximum duration");
  validatePositiveFinite(final_tolerance_rad, "final tolerance");
  for (std::size_t joint = 0; joint < qmini_arm::kJointCount; ++joint) {
    if (!std::isfinite(hard_lower_rad[joint]) ||
        !std::isfinite(hard_upper_rad[joint]) ||
        !std::isfinite(goal_position_rad[joint]) ||
        hard_lower_rad[joint] >= hard_upper_rad[joint]) {
      throw std::invalid_argument("invalid hard limits or trajectory goal");
    }
  }
  if (std::abs(trajectory.front().time_s) > 1e-9) {
    throw std::runtime_error("home trajectory must start at t=0");
  }
  if (trajectory.back().time_s > maximum_duration_s + 1e-9) {
    throw std::runtime_error("home trajectory duration exceeds the limit");
  }

  for (std::size_t sample_index = 0; sample_index < trajectory.size();
       ++sample_index) {
    const JointTrajectorySample& sample = trajectory[sample_index];
    if (!std::isfinite(sample.time_s)) {
      throw std::runtime_error("trajectory contains a non-finite time");
    }
    for (std::size_t joint = 0; joint < qmini_arm::kJointCount; ++joint) {
      const double position = sample.position_rad[joint];
      const double velocity = sample.velocity_rad_s[joint];
      if (!std::isfinite(position) || !std::isfinite(velocity)) {
        throw std::runtime_error("trajectory contains a non-finite joint value");
      }
      if (soft_lower_rad[joint] >= soft_upper_rad[joint]) {
        throw std::runtime_error("invalid soft limits");
      }
      if (allow_hard_limit_goal) {
        if (position < hard_lower_rad[joint] - 1e-8 ||
            position > hard_upper_rad[joint] + 1e-8) {
          throw std::runtime_error("trajectory joint_" +
                                   std::to_string(joint + 1) +
                                   " violates the hard limits");
        }
      } else if (position <= soft_lower_rad[joint] + limit_margin_rad ||
                 position >= soft_upper_rad[joint] - limit_margin_rad) {
        throw std::runtime_error("trajectory joint_" +
                                 std::to_string(joint + 1) +
                                 " violates the guarded soft limits");
      }
      if (std::abs(velocity) > maximum_velocity_rad_s + 1e-8) {
        throw std::runtime_error("trajectory joint_" +
                                 std::to_string(joint + 1) +
                                 " exceeds the velocity limit");
      }
    }

    if (sample_index == 0) continue;
    const JointTrajectorySample& previous = trajectory[sample_index - 1];
    const double dt = sample.time_s - previous.time_s;
    if (!(dt > 0.0) || dt > maximum_sample_period_s + 1e-8) {
      throw std::runtime_error(
          "trajectory timestamps are not strictly increasing at the required rate");
    }
    for (std::size_t joint = 0; joint < qmini_arm::kJointCount; ++joint) {
      const double sampled_acceleration =
          (sample.velocity_rad_s[joint] -
           previous.velocity_rad_s[joint]) /
          dt;
      if (std::abs(sampled_acceleration) >
          maximum_acceleration_rad_s2 + 1e-3) {
        throw std::runtime_error("trajectory joint_" +
                                 std::to_string(joint + 1) +
                                 " exceeds the acceleration limit");
      }
      const double displacement = std::abs(
          sample.position_rad[joint] - previous.position_rad[joint]);
      if (displacement > maximum_velocity_rad_s * dt + 1e-6) {
        throw std::runtime_error("trajectory position step exceeds its speed bound");
      }
    }
  }

  for (std::size_t joint = 0; joint < qmini_arm::kJointCount; ++joint) {
    if (std::abs(trajectory.front().velocity_rad_s[joint]) > 1e-8 ||
        std::abs(trajectory.back().position_rad[joint] -
                 goal_position_rad[joint]) >
            final_tolerance_rad ||
        std::abs(trajectory.back().velocity_rad_s[joint]) > 1e-8) {
      throw std::runtime_error(
          "home trajectory must start stopped and finish stopped at its goal");
    }
  }
}

}  // namespace qmini_arm
