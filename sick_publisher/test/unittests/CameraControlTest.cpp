// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <base/ErrorCode.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "MockCameraControl.hpp"
#include "MockCameraFrame.hpp"
#include "base/Result.hpp"
#include "sick_publisher/ICameraControl.hpp"

namespace sick
{
class ICameraFrame;
}

using ::testing::Return;

class ICameraControlTest : public ::testing::Test
{
protected:
  sick::MockCameraControl mockCameraControl;

  void SetUp() override {}

  void TearDown() override {}
};

TEST_F(ICameraControlTest, ConnectSuccess)
{
  EXPECT_CALL(mockCameraControl, connect()).Times(1).WillOnce(Return(sick::ErrorCode::OK));

  auto result = mockCameraControl.connect();
  EXPECT_EQ(result, sick::ErrorCode::OK);
}

TEST_F(ICameraControlTest, ConnectFailure)
{
  EXPECT_CALL(mockCameraControl, connect()).Times(1).WillOnce(Return(sick::ErrorCode::ConnectionFailed));

  auto result = mockCameraControl.connect();
  EXPECT_EQ(result, sick::ErrorCode::ConnectionFailed);
}

TEST_F(ICameraControlTest, DisconnectFailure)
{
  EXPECT_CALL(mockCameraControl, disconnect()).Times(1).WillOnce(Return(sick::ErrorCode::DisconnectionFailed));

  auto result = mockCameraControl.disconnect();
  EXPECT_EQ(result, sick::ErrorCode::DisconnectionFailed);
}

TEST_F(ICameraControlTest, GetNextFrameSuccess)
{
  std::shared_ptr<sick::ICameraFrame> mockCameraFrame = std::make_shared<sick::MockCameraFrame>();

  EXPECT_CALL(mockCameraControl, getNextFrame())
    .Times(1)
    .WillOnce(
      Return(testing::ByMove(sick::FrameOrError<std::shared_ptr<sick::ICameraFrame>>(std::move(mockCameraFrame)))));

  auto result = mockCameraControl.getNextFrame();
  EXPECT_TRUE(result.isSuccess());
  EXPECT_NE(result.getSuccess(), nullptr);
}

TEST_F(ICameraControlTest, GetCameraParameterSuccess)
{
  std::string parameterName = "ChunkScan3dAspectRatio";
  std::string mockParameterValue = "1";

  EXPECT_CALL(mockCameraControl, getCameraParameter(parameterName))
    .Times(1)
    .WillOnce(Return(testing::ByMove(sick::ParameterOrError<std::string>(std::move(mockParameterValue)))));

  auto result = mockCameraControl.getCameraParameter(parameterName);
  EXPECT_TRUE(result.isSuccess());
  EXPECT_EQ(result.getSuccess(), "1");
}

TEST_F(ICameraControlTest, LoadParamsSuccess)
{
  std::vector<std::string> gevComponents = {"Component1", "Component2"};
  const std::vector<std::pair<std::string, std::string>> gevParams = {{"Param1", "Value1"}, {"Param2", "Value2"}};

  EXPECT_CALL(mockCameraControl, loadConfig(gevComponents, gevParams)).Times(1).WillOnce(Return(sick::ErrorCode::OK));

  auto result = mockCameraControl.loadConfig(gevComponents, gevParams);
  EXPECT_EQ(result, sick::ErrorCode::OK);
}

TEST_F(ICameraControlTest, DisableFiltersSuccess)
{
  EXPECT_CALL(mockCameraControl, disableFilters()).Times(1).WillOnce(Return(sick::ErrorCode::OK));

  auto result = mockCameraControl.disableFilters();
  EXPECT_EQ(result, sick::ErrorCode::OK);
}

TEST_F(ICameraControlTest, GetComponentPixelFormatSuccess)
{
  std::string componentName = "Intensity";
  std::string mockPixelFormat = "BGR8";

  EXPECT_CALL(mockCameraControl, getComponentPixelFormat(componentName)).Times(1).WillOnce(Return(mockPixelFormat));

  auto result = mockCameraControl.getComponentPixelFormat(componentName);
  EXPECT_EQ(result, mockPixelFormat);
}