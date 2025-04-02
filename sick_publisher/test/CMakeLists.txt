# Copyright (c) 2024 SICK AG, Waldkirch
# SPDX-License-Identifier: Unlicense

find_package(ament_cmake_gmock REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

include(GoogleTest)

# Test assets
set(assets_path ${CMAKE_CURRENT_SOURCE_DIR}/test_data)
configure_file(Assets.hpp.in ${CMAKE_CURRENT_BINARY_DIR}/Assets.hpp @ONLY)

## Unittests
# These tests are expected to be executed regardless of the host environment and should run without
# any connected hardware (camera) regardless.
ament_add_gmock(${PROJECT_NAME}_unittest
  UnittestsMain.cpp)

add_subdirectory(unittests)

target_link_libraries(${PROJECT_NAME}_unittest
  sick::SICKPublisher $<$<CXX_COMPILER_ID:GNU>:gcov> GenIStream::GenIStream)

target_include_directories(${PROJECT_NAME}_unittest
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(BUILD_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_link_libraries(${PROJECT_NAME}_unittest gcov)
  target_compile_options(${PROJECT_NAME}_unittest PRIVATE
    -fprofile-arcs -ftest-coverage)
endif()

## Integrationtests
# These tests require physical access to a connected camera.
ament_add_gmock(${PROJECT_NAME}_integrationtests
  IntegrationtestsMain.cpp)

target_sources(${PROJECT_NAME}_integrationtests PUBLIC
  integrationtests/GeniStreamCameraDiscoveryTest.cpp
  integrationtests/GeniStreamCameraControlTest.cpp)

target_link_libraries(${PROJECT_NAME}_integrationtests
  sick::SICKPublisher $<$<CXX_COMPILER_ID:GNU>:gcov> GenIStream::GenIStream)

target_include_directories(${PROJECT_NAME}_integrationtests
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}_unittest std_msgs cv_bridge sensor_msgs geometry_msgs tf2)
ament_target_dependencies(${PROJECT_NAME}_integrationtests std_msgs cv_bridge)

gtest_discover_tests(${PROJECT_NAME}_unittest)
gtest_discover_tests(${PROJECT_NAME}_integrationtests)
