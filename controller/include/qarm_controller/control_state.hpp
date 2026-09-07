#ifndef QARM_CONTROLLER_CONTROL_STATE_HPP_
#define QARM_CONTROLLER_CONTROL_STATE_HPP_

#include <string>

namespace qarm_controller {

// States are deliberately independent of any vendor motor SDK.  The same
// state machine is used by the simulated and hardware controller adapters.
enum class ControllerState {
  Disconnected,
  ReadOnly,
  ZeroCapture,
  CalibrationValid,
  Ready,
  GravityHold,
  Executing,
  Fault,
  Estop,
};

inline const char* controllerStateName(ControllerState state) {
  switch (state) {
    case ControllerState::Disconnected: return "DISCONNECTED";
    case ControllerState::ReadOnly: return "READ_ONLY";
    case ControllerState::ZeroCapture: return "ZERO_CAPTURE";
    case ControllerState::CalibrationValid: return "CALIBRATION_VALID";
    case ControllerState::Ready: return "READY";
    case ControllerState::GravityHold: return "GRAVITY_HOLD";
    case ControllerState::Executing: return "EXECUTING";
    case ControllerState::Fault: return "FAULT";
    case ControllerState::Estop: return "ESTOP";
  }
  return "UNKNOWN";
}

inline bool controllerStateFromName(const std::string& name,
                                    ControllerState* result) {
  if (!result) return false;
  const ControllerState states[] = {
      ControllerState::Disconnected, ControllerState::ReadOnly,
      ControllerState::ZeroCapture, ControllerState::CalibrationValid,
      ControllerState::Ready, ControllerState::GravityHold,
      ControllerState::Executing, ControllerState::Fault,
      ControllerState::Estop};
  for (ControllerState state : states) {
    if (name == controllerStateName(state)) {
      *result = state;
      return true;
    }
  }
  return false;
}

}  // namespace qarm_controller

#endif  // QARM_CONTROLLER_CONTROL_STATE_HPP_
