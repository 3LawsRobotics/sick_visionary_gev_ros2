// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_TEST_UNITTESTS_MOCKCAMERACONTROL_HPP
#define SICK_PUBLISHER_TEST_UNITTESTS_MOCKCAMERACONTROL_HPP

#include <utility>

#include <base/ErrorCode.hpp>
#include <gmock/gmock.h>
#include <sick_publisher/ICameraControl.hpp>

namespace sick
{

class MockCameraControl : public ICameraControl
{
public:
  MOCK_METHOD(ErrorCode, connect, (), (override));
  MOCK_METHOD(ErrorCode,
              loadConfig,
              (const std::vector<std::string>&, (const std::vector<std::pair<std::string, std::string>>&)),
              (override));
  MOCK_METHOD(ErrorCode, disableFilters, (), (override));
  MOCK_METHOD(ErrorCode, disconnect, (), (override));
  MOCK_METHOD(ErrorCode, startStreaming, (), (override));
  MOCK_METHOD(ErrorCode, stopStreaming, (), (override));
  MOCK_METHOD(FrameOrError<std::shared_ptr<ICameraFrame>>, getNextFrame, (), (override));
  MOCK_METHOD(ParameterOrError<std::string>, getCameraParameter, (const std::string&), (override));
  MOCK_METHOD(ParameterOrError<std::string>, getCameraParameter, (const std::string&, const std::string&), (override));
  MOCK_METHOD(ParameterOrError<float>, getCameraParameterFloat, (const std::string&, const std::string&), (override));
  MOCK_METHOD(bool, isComponentActive, (const std::string&), (override));
  MOCK_METHOD(std::string, getComponentPixelFormat, (const std::string&), (override));
};
} // namespace sick

#endif // MOCK_CAMERA_CONTRO_HPP_