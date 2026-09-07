find_package(Threads REQUIRED)
add_library(qarm_controller src/arm_controller.cpp)
add_library(Qarm::Controller ALIAS qarm_controller)
target_include_directories(qarm_controller PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(qarm_controller PUBLIC QminiArm::Core Threads::Threads)
target_compile_options(qarm_controller PRIVATE -Wall -Wextra -Wpedantic)

if(QARM_BUILD_HARDWARE)
  add_library(qarm_controller_hardware src/motor_bus_adapter.cpp)
  add_library(Qarm::ControllerHardware ALIAS qarm_controller_hardware)
  target_include_directories(qarm_controller_hardware PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  target_link_libraries(qarm_controller_hardware PUBLIC Qarm::Controller QminiArm::Hardware)
  target_compile_options(qarm_controller_hardware PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(BUILD_TESTING AND QMINI_ARM_BUILD_TESTS)
  add_executable(qarm_controller_tests tests/controller_tests.cpp)
  target_link_libraries(qarm_controller_tests PRIVATE Qarm::Controller)
  target_compile_options(qarm_controller_tests PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME controller_tests COMMAND qarm_controller_tests)
endif()
