# Copyright (c) 2024 SICK AG, Waldkirch
# SPDX-License-Identifier: Unlicense

target_sources(${PROJECT_NAME}_unittest PUBLIC
  base/CmdLineFlagsTest.cpp
  base/ErrorTraceTest.cpp
  base/LoggingTest.cpp
  base/PlatformTest.cpp
  base/ResultTest.cpp
  ArraySizeTest.cpp
  system/EnvironmentTest.cpp
  CameraControlTest.cpp
  CameraDiscoveryTest.cpp
  CameraFrameTest.cpp
  CameraInfoTest.cpp
  CameraInfoParametersTest.cpp
  CameraIntrinsicsTest.cpp
  FrameTransformationsTest.cpp
  MockCameraControl.hpp
  MockCameraDiscovery.hpp
  MockCameraFrame.hpp
  SanityTest.cpp
  TransformUtilsTest.cpp
  ParamsFileUtilsTest.cpp
  PointCloudUtilsTest.cpp)