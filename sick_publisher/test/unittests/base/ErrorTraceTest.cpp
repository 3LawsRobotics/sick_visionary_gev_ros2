// Copyright (c) 2025 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include "base/ErrorTrace.hpp"

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "base/ErrorCode.hpp"

namespace sick
{
class ErrorCodeTest : public ::testing::Test
{
protected:
  void SetUp() override {}

  void TearDown() override {}
};

TEST_F(ErrorCodeTest, ConnectionFailedMacro)
{
  auto error = SICK_CONNECTION_FAILED("Failed to connect to camera");
  ASSERT_EQ(error->getCode(), ErrorCode::ConnectionFailed);
  ASSERT_EQ(error->getMessage(), "Failed to connect to camera");

  const auto& backtrace = error->getBacktrace();
  ASSERT_EQ(backtrace.size(), 1);
  EXPECT_STREQ(backtrace[0].file, __FILE__);
  EXPECT_STREQ(backtrace[0].function, __func__);
  EXPECT_EQ(backtrace[0].line, __LINE__ - 8);
}

TEST_F(ErrorCodeTest, FrameGrabberFailedMacro)
{
  auto error = SICK_FRAME_GRABBER_FAILED("No data available");
  ASSERT_EQ(error->getCode(), ErrorCode::NoData);
  ASSERT_EQ(error->getMessage(), "No data available");

  const auto& backtrace = error->getBacktrace();
  ASSERT_EQ(backtrace.size(), 1);
  EXPECT_STREQ(backtrace[0].file, __FILE__);
  EXPECT_STREQ(backtrace[0].function, __func__);
  EXPECT_EQ(backtrace[0].line, __LINE__ - 8);
}

TEST_F(ErrorCodeTest, InternalErrorMacro)
{
  auto error = SICK_INTERNAL_ERROR("Internal error occurred");
  ASSERT_EQ(error->getCode(), ErrorCode::InternalError);
  ASSERT_EQ(error->getMessage(), "Internal error occurred");

  const auto& backtrace = error->getBacktrace();
  ASSERT_EQ(backtrace.size(), 1);
  EXPECT_STREQ(backtrace[0].file, __FILE__);
  EXPECT_STREQ(backtrace[0].function, __func__);
  EXPECT_EQ(backtrace[0].line, __LINE__ - 8);
}

TEST_F(ErrorCodeTest, ParamFailedMacro)
{
  auto error = SICK_PARAM_FAILED("Invalid parameter");
  ASSERT_EQ(error->getCode(), ErrorCode::InvalidParameter);
  ASSERT_EQ(error->getMessage(), "Invalid parameter");

  const auto& backtrace = error->getBacktrace();
  ASSERT_EQ(backtrace.size(), 1);
  EXPECT_STREQ(backtrace[0].file, __FILE__);
  EXPECT_STREQ(backtrace[0].function, __func__);
  EXPECT_EQ(backtrace[0].line, __LINE__ - 8);
}
} // namespace sick