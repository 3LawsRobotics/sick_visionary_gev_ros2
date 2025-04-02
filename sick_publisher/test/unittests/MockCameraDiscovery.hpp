// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_TEST_UNITTESTS_MOCKCAMERADISCOVERY_HPP
#define SICK_PUBLISHER_TEST_UNITTESTS_MOCKCAMERADISCOVERY_HPP

#include <memory>
#include <vector>

#include <base/ErrorCode.hpp>
#include <gmock/gmock.h>
#include <sick_publisher/CameraInfo.hpp>
#include <sick_publisher/ICameraControl.hpp>
#include <sick_publisher/ICameraDiscovery.hpp>

namespace sick
{
class MockCameraDiscovery : public ICameraDiscovery
{
public:
  MOCK_METHOD(ErrorCode, discoverCameras, (), (override));
  MOCK_METHOD(std::vector<CameraInfo>, getDiscoveredCameras, (), (override));
  MOCK_METHOD(CameraOrError<std::shared_ptr<ICameraControl>>,
              getCameraControl,
              (const std::string& serial),
              (override));
};
} // namespace sick

#endif // MOCK_CAMERA_DISCOVERY_HPP_
