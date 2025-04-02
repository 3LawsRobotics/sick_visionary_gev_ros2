// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include <memory>

#include <gtest/gtest.h>
#include <sick_publisher/CameraIntrinsics.hpp>

TEST(CameraIntrinsicsTest, DefaultConstructor)
{
  sick::CameraIntrinsics intrinsics;

  EXPECT_FLOAT_EQ(intrinsics.focalLength, 0.0F);
  EXPECT_FLOAT_EQ(intrinsics.principalPointU, 0.0F);
  EXPECT_FLOAT_EQ(intrinsics.principalPointV, 0.0F);
  EXPECT_FLOAT_EQ(intrinsics.aspectRatio, 1.0F);
  EXPECT_FLOAT_EQ(intrinsics.scaleC, 1.0F);
  EXPECT_FLOAT_EQ(intrinsics.offset, 0.0F);
}