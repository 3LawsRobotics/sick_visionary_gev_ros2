// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include "sick_publisher/CameraInfo.hpp"

#include <string>

#include <gtest/gtest.h>

namespace sick
{
TEST(CameraInfoTest, ConstructorTest)
{
  // Arrange
  std::string id = "SICKGigEVisionTL_DEV_000677ff0001";
  std::string model = "Visionary-B Two V3S146-1x";
  std::string ipAddress = "192.168.1.1";
  std::string serialNumber = "1234567";

  // Act
  CameraInfo cameraInfo(id, model, ipAddress, serialNumber);

  // Assert
  EXPECT_EQ(cameraInfo.m_id, id);
  EXPECT_EQ(cameraInfo.m_model, model);
  EXPECT_EQ(cameraInfo.m_ipAddress, ipAddress);
  EXPECT_EQ(cameraInfo.m_serialNumber, serialNumber);
}

TEST(CameraInfoTest, DefaultValuesTest)
{
  // Arrange
  CameraInfo cameraInfo("SICKGigEVisionTL_DEV_000677ff0001", "Visionary-B Two V3S146-1x", "192.168.1.1", "1234567");

  // Assert
  EXPECT_FALSE(cameraInfo.m_isConnected); // Assuming default value is false
}
} // namespace sick