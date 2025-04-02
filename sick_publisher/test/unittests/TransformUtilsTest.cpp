// Copyright (c) 2025 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include "utils/TransformUtils.hpp"

#include <cmath>

#include <string>
#include <vector>

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace sick;

TEST(TransformUtilsTest, CreateStaticTransforms)
{
  std::string frameId = "base_link";
  std::string childFrameId = "camera_link";
  std::vector<double> translation = {1.0, 2.0, 3.0};
  std::vector<double> rotation = {45.0, 90.0, 180.0};
  rclcpp::Time timeStamp(1234567890);

  geometry_msgs::msg::TransformStamped transform =
    createStaticTransforms(frameId, childFrameId, translation, rotation, timeStamp);

  EXPECT_EQ(transform.header.stamp, timeStamp);
  EXPECT_EQ(transform.header.frame_id, frameId);
  EXPECT_EQ(transform.child_frame_id, childFrameId);

  EXPECT_EQ(transform.transform.translation.x, translation[0]);
  EXPECT_EQ(transform.transform.translation.y, translation[1]);
  EXPECT_EQ(transform.transform.translation.z, translation[2]);

  tf2::Quaternion quaternion;
  constexpr double degToRad = M_PI / 180.0;
  quaternion.setRPY(rotation[0] * degToRad, rotation[1] * degToRad, rotation[2] * degToRad);

  EXPECT_EQ(transform.transform.rotation.x, quaternion.x());
  EXPECT_EQ(transform.transform.rotation.y, quaternion.y());
  EXPECT_EQ(transform.transform.rotation.z, quaternion.z());
  EXPECT_EQ(transform.transform.rotation.w, quaternion.w());
}