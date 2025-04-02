// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <sick_publisher/ICameraFrame.hpp>

#include "MockCameraFrame.hpp"

using ::testing::Return;

TEST(CameraFrameTest, GetRangeDataSize)
{
  sick::MockCameraFrame mockFrame;
  std::shared_ptr<sick::MockFrameComponent> mockComponent = std::make_shared<sick::MockFrameComponent>();
  EXPECT_CALL(mockFrame, getRangeComponent()).Times(1).WillOnce(testing::Return(mockComponent));

  int expectedWidth = 640;
  int expectedHeight = 480;
  EXPECT_CALL(*mockComponent, getWidth()).Times(1).WillOnce(testing::Return(expectedWidth));
  EXPECT_CALL(*mockComponent, getDeliveredHeight()).Times(1).WillOnce(testing::Return(expectedHeight));

  auto component = mockFrame.getRangeComponent();
  ASSERT_EQ(component->getWidth(), expectedWidth);
  ASSERT_EQ(component->getDeliveredHeight(), expectedHeight);
}