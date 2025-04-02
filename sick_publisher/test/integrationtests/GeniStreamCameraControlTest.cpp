// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <GenIStreamSDK/CameraControl.hpp>
#include <GenIStreamSDK/CameraDiscovery.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <gtest/gtest.h>
#include <sensor_msgs/msg/imu.hpp>
#include <type_traits>

#include "base/ErrorCode.hpp"
#include "base/Result.hpp"
#include "sick_publisher/ICameraDiscovery.hpp"
#include "utils/FrameTransformations.hpp"

namespace sick
{
class ICameraFrame;
}

namespace sick
{
class ICameraControl;
}

using namespace sick;

TEST(GeniStreamCameraControlTest, ConfigCamera)
{
  auto& discovery = CameraDiscovery::getInstance();

  const std::vector<std::string> gevComponents{"Range", "Intensity", "ImuBasic"};

  std::pair<std::string, std::string> filter = std::make_pair("Scan3dDataFilterSelector", "ValidationFilter");
  std::pair<std::string, std::string> filterEnable = std::make_pair("Scan3dDataFilterEnable", "1");
  const std::vector<std::pair<std::string, std::string>> gevParams{filter, filterEnable};

  std::shared_ptr<ICameraControl> availableCamera{discovery.getCameraControl("23120015").getSuccess()};

  std::shared_ptr<CameraControl> camControlPtr = std::static_pointer_cast<CameraControl>(availableCamera);
  ASSERT_EQ(camControlPtr->connect(), ErrorCode::OK);
  ASSERT_EQ(camControlPtr->loadConfig(gevComponents, gevParams), ErrorCode::OK);
}

TEST(GeniStreamCameraControlTest, GetNextFrame)
{
  auto& discovery = CameraDiscovery::getInstance();

  std::shared_ptr<ICameraControl> availableCamera{discovery.getCameraControl("23120015").getSuccess()};
  std::shared_ptr<CameraControl> camControlPtr = std::static_pointer_cast<CameraControl>(availableCamera);
  ASSERT_EQ(camControlPtr->connect(), ErrorCode::OK);
  ASSERT_EQ(camControlPtr->startStreaming(), ErrorCode::OK);

  auto result = camControlPtr->getNextFrame();
  ASSERT_TRUE(result.isSuccess());
  ASSERT_TRUE(result.getSuccess() != nullptr);
}

TEST(GeniStreamCameraControlTest, DisableFilters)
{
  auto& discovery = CameraDiscovery::getInstance();

  std::shared_ptr<ICameraControl> availableCamera{discovery.getCameraControl("23120015").getSuccess()};
  std::shared_ptr<CameraControl> camControlPtr = std::static_pointer_cast<CameraControl>(availableCamera);

  ASSERT_EQ(camControlPtr->connect(), ErrorCode::OK);
  ASSERT_EQ(camControlPtr->disableFilters(), ErrorCode::OK);
}

TEST(GeniStreamCameraControlTest, GetIMUData)
{
  auto& discovery = CameraDiscovery::getInstance();

  std::shared_ptr<ICameraControl> availableCamera{discovery.getCameraControl("23120015").getSuccess()};
  std::shared_ptr<CameraControl> camControlPtr = std::static_pointer_cast<CameraControl>(availableCamera);
  ASSERT_EQ(camControlPtr->connect(), ErrorCode::OK);
  ASSERT_EQ(camControlPtr->startStreaming(), ErrorCode::OK);

  auto result = camControlPtr->getNextFrame();
  ASSERT_TRUE(result.isSuccess());

  std::vector<sensor_msgs::msg::Imu> imuVec = frameToImu(std::static_pointer_cast<ICameraFrame>(result.getSuccess()));

  std::cerr << "Linear Acceleration X: " << imuVec[0].linear_acceleration.x << std::endl;
  std::cerr << "Linear Acceleration Y: " << imuVec[0].linear_acceleration.y << std::endl;
  std::cerr << "Linear Acceleration Z: " << imuVec[0].linear_acceleration.z << std::endl;
  std::cerr << "Angular Velocity X: " << imuVec[0].angular_velocity.x << std::endl;
  std::cerr << "Angular Velocity Y: " << imuVec[0].angular_velocity.y << std::endl;
  std::cerr << "Angular Velocity Z: " << imuVec[0].angular_velocity.z << std::endl;
  std::cerr << "Orientation X: " << imuVec[0].orientation.x << std::endl;
  std::cerr << "Orientation Y: " << imuVec[0].orientation.y << std::endl;
  std::cerr << "Orientation Z: " << imuVec[0].orientation.z << std::endl;
  std::cerr << "Orientation W: " << imuVec[0].orientation.w << std::endl;
}