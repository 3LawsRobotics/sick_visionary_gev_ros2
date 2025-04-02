// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include "base/CmdLineFlags.hpp"

#include <cstdint>
#include <string>

#include <gtest/gtest.h>

TEST(CmdLineFlagsTest, CheckIntParsingWithEqualSign)
{
  int32_t val;
  ASSERT_TRUE(sick::parseInt32Flag("--value=123456", "value", &val));
  EXPECT_EQ(val, 123456);
}

TEST(CmdLineFlagsTest, CheckIntParsingWithWhitespace)
{
  int32_t val;
  ASSERT_TRUE(sick::parseInt32Flag("--value 123456", "value", &val));
  EXPECT_EQ(val, 123456);
}

TEST(CmdLineFlagsTest, CheckBoolParsing)
{
  bool val;
  ASSERT_TRUE(sick::parseBoolFlag("--doit true", "doit", &val));
  EXPECT_TRUE(val);
}

TEST(CmdLineFlagsTest, CheckStringParsing)
{
  std::string val;
  ASSERT_TRUE(sick::parseStringFlag("--widget=fidget", "widget", &val));
  EXPECT_EQ(val, "fidget");
}