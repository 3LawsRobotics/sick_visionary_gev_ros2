// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include <memory>
#include <string>
#include <utility>

#include <base/Result.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <sick_publisher/ICameraDiscovery.hpp>

#include "base/ErrorCode.hpp"
#include "MockCameraControl.hpp"
#include "MockCameraDiscovery.hpp"

namespace sick
{
class ICameraControl;

using ::testing::AtLeast;
using ::testing::ByMove;
using ::testing::Return;

TEST(CameraDiscoveryTest, DiscoverCamerasSuccessfully)
{
  MockCameraDiscovery mockDiscovery;
  EXPECT_CALL(mockDiscovery, discoverCameras()).Times(1).WillOnce(Return(ErrorCode::OK));

  auto error = mockDiscovery.discoverCameras();
  ASSERT_EQ(error, ErrorCode::OK);
}


TEST(CameraDiscoveryTest, ConnectToCameraSuccessfully)
{
  MockCameraDiscovery mockDiscovery;
  std::string cameraId = "camera123";
  std::shared_ptr<ICameraControl> mockCameraControl = std::make_shared<MockCameraControl>();

  EXPECT_CALL(mockDiscovery, getCameraControl(cameraId))
    .Times(1)
    .WillOnce(Return(ByMove(CameraOrError<std::shared_ptr<ICameraControl>>(std::move(mockCameraControl)))));

  auto cameraControlResult = mockDiscovery.getCameraControl(cameraId);
  ASSERT_FALSE(cameraControlResult.isError());
  ASSERT_TRUE(cameraControlResult.isSuccess());
  auto cameraControl = cameraControlResult.getSuccess();
  ASSERT_NE(cameraControl, nullptr);
}
} // namespace sick
