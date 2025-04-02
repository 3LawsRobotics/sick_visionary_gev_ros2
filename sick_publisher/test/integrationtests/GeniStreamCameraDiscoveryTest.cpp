// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include <memory>
#include <string>
#include <iostream>
#include <vector>

#include <GenIStreamSDK/CameraDiscovery.hpp>
#include <gtest/gtest.h>

#include "base/ErrorCode.hpp"
#include "sick_publisher/ICameraDiscovery.hpp"
#include "sick_publisher/CameraInfo.hpp"

namespace sick
{
class ICameraControl;
}

using namespace sick;

TEST(GeniStreamCameraDiscoveryTest, DiscoverAllCameras)
{
  auto& discovery = CameraDiscovery::getInstance();
  ErrorCode error = discovery.discoverCameras();

  ASSERT_EQ(error, sick::ErrorCode::OK);

  std::vector<sick::CameraInfo> info{discovery.getDiscoveredCameras()};
  ASSERT_NE(info.size(), 0);
  for (auto& i : info)
  {
    std::cout << "device id    : " << i.m_id << std::endl;
    std::cout << "model id     : " << i.m_model << std::endl;
    std::cout << "ip addres    : " << i.m_ipAddress << std::endl;
    std::cout << "serial number: " << i.m_serialNumber << std::endl;
  }
}

TEST(GeniStreamCameraDiscoveryTest, GetCameraControls)
{
  auto& discovery = CameraDiscovery::getInstance();
  ErrorCode error = discovery.discoverCameras();
  ASSERT_EQ(error, sick::ErrorCode::OK);

  std::string serial{"23120015"};
  std::shared_ptr<ICameraControl> availableCamera{nullptr};
  availableCamera = discovery.getCameraControl(serial).getSuccess();
  ASSERT_NE(availableCamera, nullptr);
}
