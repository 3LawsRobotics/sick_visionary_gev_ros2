// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include <memory>

#include "ArraySize.hpp"
#include <gtest/gtest.h>

TEST(ArraySizeTest, GetCompileTimeArraySize)
{
  int foobar[] = {2, 3, 5, 7, 9, 11};
  EXPECT_EQ(6, ARRAYSIZE(foobar));
}