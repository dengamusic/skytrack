cmake_minimum_required(VERSION 3.25)
project(skytrack VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(SKYTRACK_BUILD_TESTS "Build the unit tests" ON)
option(SKYTRACK_WERROR "Treat compiler warnings as errors" OFF)
option(SKYTRACK_SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(nlohmann_json 3.10 REQUIRED)

# Warning and sanitizer flags, applied to this project's code only.
add_library(skytrack_build_flags INTERFACE)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(skytrack_build_flags INTERFACE
    -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor
    # No compiler-inserted fused multiply-add, so builds at different
    # optimisation levels round (almost) the same way.
    -ffp-contract=off)
  if(SKYTRACK_WERROR)
    target_compile_options(skytrack_build_flags INTERFACE -Werror)
  endif()
  if(SKYTRACK_SANITIZE)
    target_compile_options(skytrack_build_flags INTERFACE
      -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(skytrack_build_flags INTERFACE -fsanitize=address,undefined)
  endif()
endif()

# Geodesy and random numbers, shared by the simulator and the tracker.
add_library(skytrack_common
  src/geo.cpp
  src/rng.cpp)
target_include_directories(skytrack_common PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(skytrack_common
  PUBLIC Eigen3::Eigen
  PRIVATE skytrack_build_flags)

add_library(skytrack_sim
  src/json_fields.cpp
  src/trajectory.cpp
  src/target.cpp
  src/measurements.cpp
  src/sensor.cpp
  src/sensors/radar.cpp
  src/sensors/camera.cpp
  src/sensors/rf_bearing.cpp
  src/sensors/cooperative.cpp
  src/scenario.cpp
  src/simulator.cpp
  src/run_writer.cpp)
target_link_libraries(skytrack_sim
  PUBLIC skytrack_common nlohmann_json::nlohmann_json
  PRIVATE skytrack_build_flags)

add_executable(skytrack_sim_app apps/skytrack_sim.cpp)
set_target_properties(skytrack_sim_app PROPERTIES OUTPUT_NAME skytrack_sim)
target_link_libraries(skytrack_sim_app PRIVATE skytrack_sim skytrack_build_flags)

# The tracker. It reuses the simulator's sensor descriptions (parsed from
# sensors.json) and message types, but never sees its ground truth.
add_library(skytrack_tracking
  src/tracking/assignment.cpp
  src/tracking/config.cpp
  src/tracking/covariance_intersection.cpp
  src/tracking/registration.cpp
  src/tracking/sensor_model.cpp
  src/tracking/tracker.cpp
  src/tracking/track_io.cpp)
target_link_libraries(skytrack_tracking
  PUBLIC skytrack_sim
  PRIVATE skytrack_build_flags)

add_executable(skytrack_track_app apps/skytrack_track.cpp)
set_target_properties(skytrack_track_app PROPERTIES OUTPUT_NAME skytrack_track)
target_link_libraries(skytrack_track_app PRIVATE skytrack_tracking skytrack_build_flags)

# Track-to-track fusion: a centre that fuses the tracks sensor nodes publish.
add_library(skytrack_fusion src/fusion/track_fusion.cpp)
target_link_libraries(skytrack_fusion
  PUBLIC skytrack_tracking
  PRIVATE skytrack_build_flags)

add_executable(skytrack_fuse_app apps/skytrack_fuse.cpp)
set_target_properties(skytrack_fuse_app PROPERTIES OUTPUT_NAME skytrack_fuse)
target_link_libraries(skytrack_fuse_app PRIVATE skytrack_fusion skytrack_build_flags)

# Closed-loop cueing: the tracker's output points a gimbal camera, co-simulated.
add_library(skytrack_cueing src/cueing/cueing.cpp)
target_link_libraries(skytrack_cueing
  PUBLIC skytrack_tracking
  PRIVATE skytrack_build_flags)

add_executable(skytrack_cue_app apps/skytrack_cue.cpp)
set_target_properties(skytrack_cue_app PROPERTIES OUTPUT_NAME skytrack_cue)
target_link_libraries(skytrack_cue_app PRIVATE skytrack_cueing skytrack_build_flags)

# Scoring against ground truth. Kept apart from the tracker, which must not depend on it.
add_library(skytrack_eval src/eval/metrics.cpp)
target_link_libraries(skytrack_eval
  PUBLIC skytrack_tracking
  PRIVATE skytrack_build_flags)

add_executable(skytrack_eval_app apps/skytrack_eval.cpp)
set_target_properties(skytrack_eval_app PROPERTIES OUTPUT_NAME skytrack_eval)
target_link_libraries(skytrack_eval_app PRIVATE skytrack_eval skytrack_build_flags)

if(SKYTRACK_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SYSTEM)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()
  add_executable(skytrack_tests
    tests/test_geo.cpp
    tests/test_rng.cpp
    tests/test_trajectory.cpp
    tests/test_sensors.cpp
    tests/test_simulator.cpp
    tests/test_assignment.cpp
    tests/test_fusion.cpp
    tests/test_cueing.cpp
    tests/test_tracking.cpp)
  target_link_libraries(skytrack_tests PRIVATE skytrack_eval skytrack_fusion skytrack_cueing GTest::gtest_main skytrack_build_flags)
  target_compile_definitions(skytrack_tests PRIVATE SKYTRACK_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
  include(GoogleTest)
  gtest_discover_tests(skytrack_tests)
endif()
