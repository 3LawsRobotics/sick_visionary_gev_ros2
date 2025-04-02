// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include "utils/FrameTransformations.hpp"

#include <algorithm>
#include <cstdint>

#include <geometry_msgs/msg/transform.hpp>
#include <std_msgs/msg/header.hpp>

#include "sick_publisher/ICameraFrame.hpp"

namespace sick
{

cv::Mat frameToRgb(const std::shared_ptr<ICameraFrame>& frame)
{
  const auto comp = frame->getIntensityComponent();
  const auto width = static_cast<int>(comp->getWidth());
  const auto height = static_cast<int>(comp->getDeliveredHeight());
  auto* data = reinterpret_cast<uint8_t*>(comp->getData());
  cv::Mat cameraFrame(height, width, CV_8UC3, data);

  return cameraFrame;
}

cv::Mat frameToGrayscale(const std::shared_ptr<ICameraFrame>& frame)
{
  const auto comp = frame->getIntensityComponent();
  const auto width = static_cast<int>(comp->getWidth());
  const auto height = static_cast<int>(comp->getDeliveredHeight());
  auto* data = reinterpret_cast<uint16_t*>(comp->getData());
  cv::Mat cameraFrame(height, width, CV_16UC1, data);
  return cameraFrame;
}

cv::Mat frameToDepth(const std::shared_ptr<ICameraFrame>& frame)
{
  auto comp = frame->getRangeComponent();
  const auto width = static_cast<int>(comp->getWidth());
  const auto height = static_cast<int>(comp->getDeliveredHeight());
  auto* data = reinterpret_cast<uint16_t*>(comp->getData());
  cv::Mat depthFrame(height, width, CV_16UC1, data);

  return depthFrame;
}

std::vector<sensor_msgs::msg::Imu> frameToImu(const std::shared_ptr<ICameraFrame>& frame)
{
  auto comp = frame->getImuComponent();
  auto imuH = comp->getDeliveredHeight();
  // Number of IMU readings correspond to the pseudo-image height.
  uint8_t numImuReadings = imuH;

  // The pseudo-image 8-bit "pixels" represent bytes of the IMU data blob delivered in this block.
  //
  // Lines of the pseudo-image represent individual IMU readings in this block, one line per reading.
  // There might be different amount of IMU readings (pseudo image lines) in each block, depending
  // how many of them were generated between the previous and current range data acquisition.
  // Note that in a corner case when no IMU readings were available, there will be a single line
  // in the component with all zeroes - however, this is just a hypothetical corner case, because
  // IMU readings are generated with higher frequency than the range sensor acquisitions.//
  // Individual IMU readings contain following values:
  // 3 x acceleration (x/y/z axis) as 64-bit float [m/s^2]
  // 3 x angular velocity (x/y/z axis) as 64-bit float [rad/s]
  // 3 x magnetic field (x/y/z axis) as 64-bit float [T]
  // 4 x orientation (unit/norm-1 quaternion) as 64-bit float
  // 1 x timestamp as 64-bit integer [ns]
  // The single IMU reading (pseudo-image "line pixels") therefore packs exactly these values.

  auto* imuDataValues = reinterpret_cast<ImuData*>(comp->getData());

  std::vector<sensor_msgs::msg::Imu> imuVec{};
  imuVec.reserve(numImuReadings);

  for (uint8_t i = 0; i < numImuReadings; ++i)
  {
    auto& imuValue = imuDataValues[i];
    sensor_msgs::msg::Imu imuMsg{};
    // quaternion
    imuMsg.orientation.x = imuValue.orientation[0];
    imuMsg.orientation.y = imuValue.orientation[1];
    imuMsg.orientation.z = imuValue.orientation[2];
    imuMsg.orientation.w = imuValue.orientation[3];
    // angular velocity
    imuMsg.angular_velocity.x = imuValue.angularVelocity[0];
    imuMsg.angular_velocity.y = imuValue.angularVelocity[1];
    imuMsg.angular_velocity.z = imuValue.angularVelocity[2];
    // linear acceleration
    imuMsg.linear_acceleration.x = imuValue.acceleration[0];
    imuMsg.linear_acceleration.y = imuValue.acceleration[1];
    imuMsg.linear_acceleration.z = imuValue.acceleration[2];

    uint64_t timeStampNs = imuValue.timeStamp;
    // Convert to seconds and nanoseconds
    int32_t sec = timeStampNs / 1000000000UL;
    uint32_t nsec = timeStampNs % 1000000000UL;

    imuMsg.header.stamp.sec = sec;
    imuMsg.header.stamp.nanosec = nsec;

    imuVec.push_back(imuMsg);
  }

  return imuVec;
}

} // namespace sick