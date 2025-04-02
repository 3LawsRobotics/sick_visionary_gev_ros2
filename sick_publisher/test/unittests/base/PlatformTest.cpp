// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include "base/Platform.hpp"

#include <memory>

#include <gtest/gtest.h>

TEST(PlatformTest, Test64Bit)
{
#if defined(SICK_OS_LINUX)
  EXPECT_EQ(sick::getPlatformString(), "linux_x64");
#else
  EXPECT_EQ(sick::getPlatformString(), "windows_x64");
#endif
}
