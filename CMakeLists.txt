cmake_minimum_required(VERSION 3.10)
project(QminiUnitreeArm VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(QARM_BUILD_HARDWARE "Build the Unitree bus and controller adapter" OFF)
option(QARM_BUILD_MAINTENANCE_APPS "Build legacy standalone motor maintenance tools" OFF)
option(QMINI_ARM_BUILD_APPS "Deprecated alias for QARM_BUILD_MAINTENANCE_APPS" OFF)
option(QMINI_ARM_BUILD_TESTS "Build offline unit tests" ON)
if(QMINI_ARM_BUILD_APPS)
  message(DEPRECATION "Use QARM_BUILD_MAINTENANCE_APPS for legacy motor tools")
  set(QARM_BUILD_MAINTENANCE_APPS ON)
endif()

add_library(qmini_arm_core
  src/gravity_control.cpp
  src/gravity_model.cpp
  src/joint_trajectory.cpp
  src/joint_conversion.cpp
  src/safety.cpp
  src/sine_trajectory.cpp
)
add_library(QminiArm::Core ALIAS qmini_arm_core)

target_include_directories(qmini_arm_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)
target_compile_options(qmini_arm_core PRIVATE -Wall -Wextra -Wpedantic)

if(QARM_BUILD_HARDWARE OR QARM_BUILD_MAINTENANCE_APPS)
  if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR
      "Unitree hardware applications require Linux. "
      "Disable QARM_BUILD_HARDWARE and QARM_BUILD_MAINTENANCE_APPS for portable tests.")
  endif()
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
  include(UnitreeActuatorSDK)
  add_library(qmini_arm_hardware src/motor_bus.cpp)
  add_library(QminiArm::Hardware ALIAS qmini_arm_hardware)
  target_link_libraries(qmini_arm_hardware
    PUBLIC QminiArm::Core
    PRIVATE Unitree::ActuatorSDK)
  target_compile_options(qmini_arm_hardware PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(QARM_BUILD_MAINTENANCE_APPS)
  add_executable(qmini_motor_state apps/read_motor_state.cpp)
  target_include_directories(qmini_motor_state PRIVATE apps)
  target_link_libraries(qmini_motor_state PRIVATE QminiArm::Hardware)
  target_compile_options(qmini_motor_state PRIVATE -Wall -Wextra -Wpedantic)

  add_executable(qmini_sine_position apps/sine_position_test.cpp)
  target_include_directories(qmini_sine_position PRIVATE apps)
  target_link_libraries(qmini_sine_position PRIVATE QminiArm::Hardware)
  target_compile_options(qmini_sine_position PRIVATE -Wall -Wextra -Wpedantic)

  add_executable(qmini_gravity_comp apps/gravity_compensation.cpp)
  target_include_directories(qmini_gravity_comp PRIVATE apps)
  target_link_libraries(qmini_gravity_comp PRIVATE QminiArm::Hardware)
  target_compile_options(qmini_gravity_comp PRIVATE -Wall -Wextra -Wpedantic)

  add_executable(qmini_return_to_zero apps/return_to_zero.cpp)
  target_include_directories(qmini_return_to_zero PRIVATE apps)
  target_link_libraries(qmini_return_to_zero PRIVATE QminiArm::Hardware)
  target_compile_options(qmini_return_to_zero PRIVATE -Wall -Wextra -Wpedantic)

  add_executable(m8010_readonly tools/m8010_readonly.cpp)
  target_link_libraries(m8010_readonly PRIVATE Unitree::ActuatorSDK)
  target_compile_options(m8010_readonly PRIVATE -Wall -Wextra -Wpedantic)

  # The Unitree SDK is a prebuilt shared library. Keep build-tree executables
  # runnable without copying or modifying anything in the SDK repository.
  set_target_properties(
    qmini_motor_state qmini_sine_position qmini_gravity_comp
    qmini_return_to_zero m8010_readonly PROPERTIES
    BUILD_RPATH "${UNITREE_ACTUATOR_SDK_LIBRARY_DIR}"
  )
  # Legacy tools are build-tree maintenance utilities, never production installs.
endif()

include(CTest)
add_subdirectory(controller)
if(BUILD_TESTING AND QMINI_ARM_BUILD_TESTS)
  add_executable(qmini_arm_core_tests tests/core_tests.cpp)
  target_link_libraries(qmini_arm_core_tests PRIVATE QminiArm::Core)
  target_compile_options(qmini_arm_core_tests PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME core_tests COMMAND qmini_arm_core_tests)

  if(QARM_BUILD_MAINTENANCE_APPS)
    add_test(NAME motor_state_help COMMAND qmini_motor_state --help)
    add_test(NAME sine_help COMMAND qmini_sine_position --help)
    add_test(NAME sine_dry_run COMMAND qmini_sine_position --dry-run)
    add_test(NAME gravity_help COMMAND qmini_gravity_comp --help)
    add_test(NAME return_to_zero_help COMMAND qmini_return_to_zero --help)
    add_test(NAME m8010_readonly_help COMMAND m8010_readonly --help)
    add_test(NAME gravity_dry_run
             COMMAND qmini_gravity_comp
                     --config ${CMAKE_CURRENT_SOURCE_DIR}/config/gravity_comp.conf
                     --dry-run)
    add_test(NAME return_to_zero_dry_run
             COMMAND qmini_return_to_zero
                     --config ${CMAKE_CURRENT_SOURCE_DIR}/config/gravity_comp.conf
                     --trajectory ${CMAKE_CURRENT_SOURCE_DIR}/tests/data/calibration_home.csv
                     --dry-run)
    add_test(NAME sine_reject_duplicate_ids
             COMMAND qmini_sine_position --dry-run --ids 0,1,1)
    set_tests_properties(sine_reject_duplicate_ids PROPERTIES WILL_FAIL TRUE)
  endif()
endif()
