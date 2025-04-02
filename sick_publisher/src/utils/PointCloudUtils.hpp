// Copyright (c) 2025 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_SRC_UTILS_POINTCLOUDUTILS_HPP
#define SICK_PUBLISHER_SRC_UTILS_POINTCLOUDUTILS_HPP

#include <cstdint>
#include <vector>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace sensor_msgs
{
class PointCloud2Modifier;
}

namespace sick
{
/**
 * @brief Converts an RGB image to a point cloud.
 * 
 * @param intensityImg The intensity image (RGB).
 * @param depthPtr Pointer to the depth data.
 * @param height Height of the image.
 * @param width Width of the image.
 * @param cachedX Cached X coordinates.
 * @param cachedY Cached Y coordinates.
 * @param scaleC Scaling factor for the depth data.
 * @param offset Offset for the depth data.
 * @param pointCloudMsg The resulting point cloud message.
 * @param modifier Modifier for the point cloud message.
 */
void rgbToPointcloud(const sensor_msgs::msg::Image::ConstSharedPtr& intensityImg,
                     const float* depthPtr,
                     uint32_t height,
                     uint32_t width,
                     const std::vector<float>& cachedX,
                     const std::vector<float>& cachedY,
                     float scaleC,
                     float offset,
                     sensor_msgs::msg::PointCloud2& pointCloudMsg,
                     sensor_msgs::PointCloud2Modifier& modifier);

/**
 * @brief Converts a grayscale image to a point cloud.
 * 
 * @param intensityImg The intensity image (grayscale).
 * @param depthPtr Pointer to the depth data.
 * @param height Height of the image.
 * @param width Width of the image.
 * @param cachedX Cached X coordinates.
 * @param cachedY Cached Y coordinates.
 * @param scaleC Scaling factor for the depth data.
 * @param offset Offset for the depth data.
 * @param pointCloudMsg The resulting point cloud message.
 * @param modifier Modifier for the point cloud message.
 */
void grayscaleToPointcloud(const sensor_msgs::msg::Image::ConstSharedPtr& intensityImg,
                           const float* depthPtr,
                           uint32_t height,
                           uint32_t width,
                           const std::vector<float>& cachedX,
                           const std::vector<float>& cachedY,
                           float scaleC,
                           float offset,
                           sensor_msgs::msg::PointCloud2& pointCloudMsg,
                           sensor_msgs::PointCloud2Modifier& modifier);
} // namespace sick
#endif // SICK_PUBLISHER_SRC_UTILS_POINTCLOUDUTILS_HPP