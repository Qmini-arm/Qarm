# Locate the adjacent, vendor-supplied Unitree actuator SDK without editing it.
get_filename_component(
  _QMINI_DEFAULT_SDK_ROOT
  "${CMAKE_CURRENT_LIST_DIR}/../../unitree_actuator_sdk"
  ABSOLUTE
)
set(
  UNITREE_ACTUATOR_SDK_ROOT
  "${_QMINI_DEFAULT_SDK_ROOT}"
  CACHE PATH
  "Path to unitree_actuator_sdk"
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(_QMINI_UNITREE_LIBRARY_NAME libUnitreeMotorSDK_Arm64.so)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
  set(_QMINI_UNITREE_LIBRARY_NAME libUnitreeMotorSDK_Linux64.so)
else()
  message(FATAL_ERROR
    "Unsupported architecture '${CMAKE_SYSTEM_PROCESSOR}'; the bundled SDK "
    "only contains x86_64 and arm64 control libraries"
  )
endif()

set(UNITREE_ACTUATOR_SDK_INCLUDE_DIR
    "${UNITREE_ACTUATOR_SDK_ROOT}/include")
set(UNITREE_ACTUATOR_SDK_LIBRARY_DIR
    "${UNITREE_ACTUATOR_SDK_ROOT}/lib")
set(UNITREE_ACTUATOR_SDK_LIBRARY
    "${UNITREE_ACTUATOR_SDK_LIBRARY_DIR}/${_QMINI_UNITREE_LIBRARY_NAME}")

if(NOT EXISTS
   "${UNITREE_ACTUATOR_SDK_INCLUDE_DIR}/unitreeMotor/unitreeMotor.h")
  message(FATAL_ERROR
    "Unitree SDK headers not found under ${UNITREE_ACTUATOR_SDK_INCLUDE_DIR}"
  )
endif()
if(NOT EXISTS "${UNITREE_ACTUATOR_SDK_LIBRARY}")
  message(FATAL_ERROR
    "Unitree SDK library not found: ${UNITREE_ACTUATOR_SDK_LIBRARY}"
  )
endif()

if(NOT TARGET Unitree::ActuatorSDK)
  add_library(Unitree::ActuatorSDK SHARED IMPORTED)
  set_target_properties(Unitree::ActuatorSDK PROPERTIES
    IMPORTED_LOCATION "${UNITREE_ACTUATOR_SDK_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${UNITREE_ACTUATOR_SDK_INCLUDE_DIR}"
  )
endif()

