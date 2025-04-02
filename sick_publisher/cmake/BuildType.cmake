# Copyright (c) 2024 SICK AG, Waldkirch
# SPDX-License-Identifier: Unlicense

set(sick_visionary_gev_ros2_SUPPORTED_BUILD_TYPES "Debug" "Coverage" "Release" "RelWithDebInfo")

set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING
  "Select build type, options are: ${sick_visionary_gev_ros2_SUPPORTED_BUILD_TYPES}" FORCE)

# Set default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Debug' as none was specified.")
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${sick_visionary_gev_ros2_SUPPORTED_BUILD_TYPES})

if (NOT ${CMAKE_BUILD_TYPE} IN_LIST sick_visionary_gev_ros2_SUPPORTED_BUILD_TYPES)
  message(FATAL_ERROR "Unsupported build type: ${CMAKE_BUILD_TYPE}")
endif()