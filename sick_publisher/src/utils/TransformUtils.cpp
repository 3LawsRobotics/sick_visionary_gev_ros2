// Copyright (c) 2025 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include "TransformUtils.hpp"

#include <cmath>

#include <geometry_msgs/msg/transform.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace sick
{
geometry_msgs::msg::TransformStamped createStaticTransforms(const std::string& frameId,
                                                            const std::string& childFrameId,
                                                            const std::vector<double> translation,
                                                            const std::vector<double> rotation,
                                                            const rclcpp::Time& timeStamp)
{
  geometry_msgs::msg::TransformStamped staticTransform;
  staticTransform.header.stamp = timeStamp;
  staticTransform.header.frame_id = frameId;
  staticTransform.child_frame_id = childFrameId;

  staticTransform.transform.translation.x = translation[0];
  staticTransform.transform.translation.y = translation[1];
  staticTransform.transform.translation.z = translation[2];

  tf2::Quaternion quaternion;
  constexpr double degToRad = M_PI / 180.0;
  quaternion.setRPY(rotation[0] * degToRad, rotation[1] * degToRad, rotation[2] * degToRad);
  staticTransform.transform.rotation.x = quaternion.x();
  staticTransform.transform.rotation.y = quaternion.y();
  staticTransform.transform.rotation.z = quaternion.z();
  staticTransform.transform.rotation.w = quaternion.w();

  return staticTransform;
}

} // namespace sick