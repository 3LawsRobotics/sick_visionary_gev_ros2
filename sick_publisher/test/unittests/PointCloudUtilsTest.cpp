// Copyright (c) 2025 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include "utils/PointCloudUtils.hpp"

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>
#include <std_msgs/msg/header.hpp>

#ifdef ROS2_HUMBLE
  #include <cv_bridge/cv_bridge.h>
#else
  #include <cv_bridge/cv_bridge.hpp>
#endif

#include <gtest/gtest.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using namespace sick;

TEST(PointCloudUtilsTest, RgbToPointcloud)
{
  // Create dummy RGB image
  cv::Mat rgbImage(2, 2, CV_8UC3, cv::Scalar(255, 0, 0)); // Red image
  sensor_msgs::msg::Image::SharedPtr intensityImg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rgbImage).toImageMsg();

  // Create dummy depth data
  std::vector<float> depthData = {1.0, 2.0, 3.0, 4.0};
  const float* depthPtr = depthData.data();

  // Create cached X and Y data
  std::vector<float> cachedX = {0.5, 0.5, 0.5, 0.5};
  std::vector<float> cachedY = {0.5, 0.5, 0.5, 0.5};

  // Create PointCloud2 message and modifier
  sensor_msgs::msg::PointCloud2 pointCloudMsg;
  pointCloudMsg.height = 2;
  pointCloudMsg.width = 2;
  pointCloudMsg.is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(pointCloudMsg);

  // Call the function
  rgbToPointcloud(intensityImg, depthPtr, 2, 2, cachedX, cachedY, 1.0, 0.0, pointCloudMsg, modifier);

  // Verify the pointcloud data
  sensor_msgs::PointCloud2Iterator<float> iX(pointCloudMsg, "x");
  sensor_msgs::PointCloud2Iterator<float> iY(pointCloudMsg, "y");
  sensor_msgs::PointCloud2Iterator<float> iZ(pointCloudMsg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iR(pointCloudMsg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iG(pointCloudMsg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iB(pointCloudMsg, "b");

  for (uint32_t i = 0; i < 4; ++i, ++iX, ++iY, ++iZ, ++iR, ++iG, ++iB)
  {
    EXPECT_FLOAT_EQ(*iX, cachedX[i] * depthData[i]);
    EXPECT_FLOAT_EQ(*iY, cachedY[i] * depthData[i]);
    EXPECT_FLOAT_EQ(*iZ, depthData[i]);
    EXPECT_EQ(*iR, 0);   // Red
    EXPECT_EQ(*iG, 0);   // Green
    EXPECT_EQ(*iB, 255); // Blue
  }
}

TEST(PointCloudUtilsTest, GrayscaleToPointcloud)
{
  // Create dummy grayscale image
  cv::Mat grayscaleImage(2, 2, CV_16UC1, cv::Scalar(100)); // Intensity 100
  sensor_msgs::msg::Image::SharedPtr intensityImg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", grayscaleImage).toImageMsg();

  // Create dummy depth data
  std::vector<float> depthData = {1.0, 2.0, 3.0, 4.0};
  const float* depthPtr = depthData.data();

  // Create cached X and Y data
  std::vector<float> cachedX = {0.5, 0.5, 0.5, 0.5};
  std::vector<float> cachedY = {0.5, 0.5, 0.5, 0.5};

  // Create PointCloud2 message and modifier
  sensor_msgs::msg::PointCloud2 pointCloudMsg;
  pointCloudMsg.height = 2;
  pointCloudMsg.width = 2;
  pointCloudMsg.is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(pointCloudMsg);

  // Call the function
  grayscaleToPointcloud(intensityImg, depthPtr, 2, 2, cachedX, cachedY, 1.0, 0.0, pointCloudMsg, modifier);

  // Verify the pointcloud data
  sensor_msgs::PointCloud2Iterator<float> iX(pointCloudMsg, "x");
  sensor_msgs::PointCloud2Iterator<float> iY(pointCloudMsg, "y");
  sensor_msgs::PointCloud2Iterator<float> iZ(pointCloudMsg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iI(pointCloudMsg, "Intensity");

  for (uint32_t i = 0; i < 4; ++i, ++iX, ++iY, ++iZ, ++iI)
  {
    EXPECT_FLOAT_EQ(*iX, cachedX[i] * depthData[i]);
    EXPECT_FLOAT_EQ(*iY, cachedY[i] * depthData[i]);
    EXPECT_FLOAT_EQ(*iZ, depthData[i]);
    EXPECT_EQ(*iI, 100); // Intensity
  }
}