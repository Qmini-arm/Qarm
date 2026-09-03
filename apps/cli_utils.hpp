#ifndef QMINI_ARM_APPS_CLI_UTILS_HPP_
#define QMINI_ARM_APPS_CLI_UTILS_HPP_

#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace qmini_arm {
namespace cli {

inline std::string requireValue(int argc, char** argv, int& index,
                                const std::string& option) {
  if (index + 1 >= argc) {
    throw std::runtime_error("missing value for " + option);
  }
  ++index;
  return argv[index];
}

inline double parseDouble(const std::string& option,
                          const std::string& value) {
  std::size_t used = 0;
  double result = 0.0;
  try {
    result = std::stod(value, &used);
  } catch (const std::exception&) {
    throw std::runtime_error(option + " requires a number, got: " + value);
  }
  if (used != value.size() || !std::isfinite(result)) {
    throw std::runtime_error(option + " requires a finite number, got: " +
                             value);
  }
  return result;
}

inline int parseInt(const std::string& option, const std::string& value) {
  std::size_t used = 0;
  long result = 0;
  try {
    result = std::stol(value, &used, 10);
  } catch (const std::exception&) {
    throw std::runtime_error(option + " requires an integer, got: " + value);
  }
  if (used != value.size() || result < std::numeric_limits<int>::min() ||
      result > std::numeric_limits<int>::max()) {
    throw std::runtime_error(option + " requires an integer, got: " + value);
  }
  return static_cast<int>(result);
}

inline std::vector<int> parseIds(const std::string& value) {
  if (value.empty()) {
    throw std::runtime_error("--ids requires a comma-separated ID list");
  }
  std::vector<int> ids;
  std::size_t begin = 0;
  while (begin <= value.size()) {
    const std::size_t end = value.find(',', begin);
    const std::string token = value.substr(begin, end - begin);
    if (token.empty()) {
      throw std::runtime_error("--ids contains an empty ID: " + value);
    }
    ids.push_back(parseInt("--ids", token));
    if (end == std::string::npos) break;
    begin = end + 1;
  }
  return ids;
}

inline void validateIds(const std::vector<int>& ids) {
  if (ids.empty() || ids.size() > 15) {
    throw std::runtime_error("motor ID list must contain 1..15 entries");
  }
  bool used[15] = {};
  for (const int id : ids) {
    if (id < 0 || id > 14) {
      throw std::runtime_error(
          "motor ID must be 0..14; ID 15 is broadcast/no reply");
    }
    if (used[id]) {
      throw std::runtime_error("duplicate motor ID: " + std::to_string(id));
    }
    used[id] = true;
  }
}

inline std::string formatIds(const std::vector<int>& ids) {
  std::ostringstream output;
  for (std::size_t i = 0; i < ids.size(); ++i) {
    if (i > 0) output << ',';
    output << ids[i];
  }
  return output.str();
}

}  // namespace cli
}  // namespace qmini_arm

#endif  // QMINI_ARM_APPS_CLI_UTILS_HPP_

