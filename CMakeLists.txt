cmake_minimum_required(VERSION 3.10)
project(QminiUnitreeArm VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(QMINI_ARM_BUILD_APPS "Build command-line applications" ON)
option(QMINI_ARM_BUILD_TESTS "Build offline unit tests" ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(UnitreeActuatorSDK)

add_library(qmini_arm_core
  src/joint_conversion.cpp
  src/motor_bus.cpp
  src/safety.cpp
  src/sine_trajectory.cpp
)
add_library(QminiArm::Core ALIAS qmini_arm_core)

target_include_directories(qmini_arm_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  PRIVATE
    "${UNITREE_ACTUATOR_SDK_INCLUDE_DIR}"
)
target_link_libraries(qmini_arm_core PRIVATE Unitree::ActuatorSDK)
target_compile_options(qmini_arm_core PRIVATE -Wall -Wextra -Wpedantic)

if(QMINI_ARM_BUILD_APPS)
  add_executable(qmini_motor_state apps/read_motor_state.cpp)
  target_include_directories(qmini_motor_state PRIVATE apps)
  target_link_libraries(qmini_motor_state PRIVATE QminiArm::Core)
  target_compile_options(qmini_motor_state PRIVATE -Wall -Wextra -Wpedantic)

  add_executable(qmini_sine_position apps/sine_position_test.cpp)
  target_include_directories(qmini_sine_position PRIVATE apps)
  target_link_libraries(qmini_sine_position PRIVATE QminiArm::Core)
  target_compile_options(qmini_sine_position PRIVATE -Wall -Wextra -Wpedantic)

  # The Unitree SDK is a prebuilt shared library. Keep build-tree executables
  # runnable without copying or modifying anything in the SDK repository.
  set_target_properties(qmini_motor_state qmini_sine_position PROPERTIES
    BUILD_RPATH "${UNITREE_ACTUATOR_SDK_LIBRARY_DIR}"
  )
endif()

include(CTest)
if(BUILD_TESTING AND QMINI_ARM_BUILD_TESTS)
  add_executable(qmini_arm_core_tests tests/core_tests.cpp)
  target_link_libraries(qmini_arm_core_tests PRIVATE QminiArm::Core)
  target_compile_options(qmini_arm_core_tests PRIVATE -Wall -Wextra -Wpedantic)
  set_target_properties(qmini_arm_core_tests PROPERTIES
    BUILD_RPATH "${UNITREE_ACTUATOR_SDK_LIBRARY_DIR}"
  )
  add_test(NAME core_tests COMMAND qmini_arm_core_tests)

  if(QMINI_ARM_BUILD_APPS)
    add_test(NAME motor_state_help COMMAND qmini_motor_state --help)
    add_test(NAME sine_help COMMAND qmini_sine_position --help)
    add_test(NAME sine_dry_run COMMAND qmini_sine_position --dry-run)
    add_test(NAME sine_reject_duplicate_ids
             COMMAND qmini_sine_position --dry-run --ids 0,1,1)
    set_tests_properties(sine_reject_duplicate_ids PROPERTIES WILL_FAIL TRUE)
  endif()
endif()

