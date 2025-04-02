// Copyright (c) 2025 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include "PointCloudUtils.hpp"

#include <memory>

#ifdef ROS2_HUMBLE
  #include <cv_bridge/cv_bridge.h>
#else
  #include <cv_bridge/cv_bridge.hpp>
#endif
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace sick
{
void rgbToPointcloud(const sensor_msgs::msg::Image::ConstSharedPtr& intensityImg,
                     const float* depthPtr,
                     uint32_t height,
                     uint32_t width,
                     const std::vector<float>& cachedX,
                     const std::vector<float>& cachedY,
                     float scaleC,
                     float offset,
                     sensor_msgs::msg::PointCloud2& pointCloudMsg,
                     sensor_msgs::PointCloud2Modifier& modifier)
{
  // Convert ROS images to OpenCV images
  const cv::Mat rgbImage = cv_bridge::toCvShare(intensityImg)->image;

  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  sensor_msgs::PointCloud2Iterator<float> iX(pointCloudMsg, "x");
  sensor_msgs::PointCloud2Iterator<float> iY(pointCloudMsg, "y");
  sensor_msgs::PointCloud2Iterator<float> iZ(pointCloudMsg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iR(pointCloudMsg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iG(pointCloudMsg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iB(pointCloudMsg, "b");

  const uint8_t* rgbPtr = rgbImage.ptr<uint8_t>();

  for (uint32_t v = 0; v < height; ++v)
  {
    for (uint32_t u = 0; u < width; ++u, ++iX, ++iY, ++iZ, ++iR, ++iG, ++iB)
    {
      const auto idx = v * width + u;
      const float z = depthPtr[idx] * scaleC + offset;

      *iX = cachedX[idx] * z;
      *iY = cachedY[idx] * z;
      *iZ = z;

      // Set RGB values from rgbImage
      *iR = rgbPtr[3 * idx + 2]; // R
      *iG = rgbPtr[3 * idx + 1]; // G
      *iB = rgbPtr[3 * idx + 0]; // B
    }
  }
}

void grayscaleToPointcloud(const sensor_msgs::msg::Image::ConstSharedPtr& intensityImg,
                           const float* depthPtr,
                           uint32_t height,
                           uint32_t width,
                           const std::vector<float>& cachedX,
                           const std::vector<float>& cachedY,
                           float scaleC,
                           float offset,
                           sensor_msgs::msg::PointCloud2& pointCloudMsg,
                           sensor_msgs::PointCloud2Modifier& modifier)
{
  // Convert ROS images to OpenCV images
  const cv::Mat grayscaleImage = cv_bridge::toCvShare(intensityImg)->image;

  modifier.setPointCloud2Fields(4,
                                "x",
                                1,
                                sensor_msgs::msg::PointField::FLOAT32,
                                "y",
                                1,
                                sensor_msgs::msg::PointField::FLOAT32,
                                "z",
                                1,
                                sensor_msgs::msg::PointField::FLOAT32,
                                "Intensity",
                                1,
                                sensor_msgs::msg::PointField::UINT16);

  sensor_msgs::PointCloud2Iterator<float> iX(pointCloudMsg, "x");
  sensor_msgs::PointCloud2Iterator<float> iY(pointCloudMsg, "y");
  sensor_msgs::PointCloud2Iterator<float> iZ(pointCloudMsg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iI(pointCloudMsg, "Intensity");

  const uint16_t* intensityPtr = grayscaleImage.ptr<uint16_t>();

  for (uint32_t v = 0; v < height; ++v)
  {
    for (uint32_t u = 0; u < width; ++u, ++iX, ++iY, ++iZ, ++iI)
    {
      const auto idx = v * width + u;
      const float z = depthPtr[idx] * scaleC + offset;

      *iX = cachedX[idx] * z;
      *iY = cachedY[idx] * z;
      *iZ = z;

      // Set intensity values
      *iI = intensityPtr[idx];
    }
  }
}
} // namespace sick