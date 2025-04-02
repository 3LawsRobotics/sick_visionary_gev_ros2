// Copyright (c) 2025 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_SRC_UTILS_TRANSFORMUTILS_HPP
#define SICK_PUBLISHER_SRC_UTILS_TRANSFORMUTILS_HPP

#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace rclcpp
{
class Time;
}

namespace sick
{
/// @brief Creates static transforms.
/// @param frameId The frame ID.
/// @param childFrameId The child frame ID.
/// @param translation The translation vector.
/// @param rotation The rotation vector with angles in [degree].
/// @param timeStamp The timestamp acquired from the frame.
/// @return The transform stamped message.
geometry_msgs::msg::TransformStamped createStaticTransforms(const std::string& frameId,
                                                            const std::string& childFrameId,
                                                            const std::vector<double> translation,
                                                            const std::vector<double> rotation,
                                                            const rclcpp::Time& timeStamp);
} // namespace sick
#endif // TRANSFORM_UTILS_HPP