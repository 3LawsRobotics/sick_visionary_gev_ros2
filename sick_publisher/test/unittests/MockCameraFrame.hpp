// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_TEST_UNITTESTS_MOCKCAMERAFRAME_HPP
#define SICK_PUBLISHER_TEST_UNITTESTS_MOCKCAMERAFRAME_HPP

#ifdef ROS2_HUMBLE
  #include <cv_bridge/cv_bridge.h>
#else
  #include <cv_bridge/cv_bridge.hpp>
#endif
#include <gmock/gmock.h>
#include <sick_publisher/ICameraFrame.hpp>

namespace sick
{

class MockFrameComponent : public IFrameComponent
{
public:
  MOCK_METHOD(int, getWidth, (), (const, override));
  MOCK_METHOD(int, getDeliveredHeight, (), (const, override));
  MOCK_METHOD(void*, getData, (), (const, override));
};

class MockCameraFrame : public ICameraFrame
{
public:
  MOCK_METHOD(std::shared_ptr<IFrameComponent>, getRangeComponent, (), (const, override));
  MOCK_METHOD(std::shared_ptr<IFrameComponent>, getIntensityComponent, (), (const, override));
  MOCK_METHOD(std::shared_ptr<IFrameComponent>, getDepthComponent, (), (const, override));
  MOCK_METHOD(std::shared_ptr<IFrameComponent>, getImuComponent, (), (const, override));
  MOCK_METHOD(std::string, getFrameID, (), (const, override));
  MOCK_METHOD(uint64_t, getTimestamp, (), (const, override));
};
} // namespace sick

#endif // MOCK_CAMERA_FRAME_HPP_