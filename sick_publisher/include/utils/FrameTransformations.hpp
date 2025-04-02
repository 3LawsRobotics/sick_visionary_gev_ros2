// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_INCLUDE_UTILS_FRAMETRANSFORMATIONS_HPP
#define SICK_PUBLISHER_INCLUDE_UTILS_FRAMETRANSFORMATIONS_HPP

#include <cstdint>
#include <memory>
#include <vector>

#include <opencv2/core.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace sick
{

struct ImuData
{
  double acceleration[3];
  double angularVelocity[3];
  double magneticField[3];
  double orientation[4];
  uint64_t timeStamp;
};

class ICameraFrame;

/// @brief Converts a camera frame to an RGB image.
/// @param frame A shared pointer to the camera frame.
/// @return cv::Mat The RGB image.
cv::Mat frameToRgb(const std::shared_ptr<ICameraFrame>& frame);

/// @brief Converts a camera frame to a grayscale image.
/// @param frame A shared pointer to the camera frame.
/// @return cv::Mat The grayscale image.
cv::Mat frameToGrayscale(const std::shared_ptr<ICameraFrame>& frame);

/// @brief Converts a camera frame to a depth image.
/// @param frame A shared pointer to the camera frame.
/// @return cv::Mat The depth image.
cv::Mat frameToDepth(const std::shared_ptr<ICameraFrame>& frame);

/// @brief Converts a camera frame to a ROS IMU message.
/// @param frame A shared pointer to the camera frame.
/// @return std::vector<sensor_msgs::msg::Imu> Vector of ROS IMU messages.
std::vector<sensor_msgs::msg::Imu> frameToImu(const std::shared_ptr<ICameraFrame>& frame);
} // namespace sick

#endif // FRAMETRANSFORMATIONS_HPP_HPP_
