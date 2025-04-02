// Copyright (c) 2025 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include "utils/FrameTransformations.hpp"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/transform.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/header.hpp>

#include "MockCameraFrame.hpp"

using namespace sick;
using ::testing::_;
using ::testing::Return;

class FrameTransformationsTest : public ::testing::Test
{
protected:
  std::shared_ptr<MockCameraFrame> mockFrame = std::make_shared<MockCameraFrame>();
};

TEST_F(FrameTransformationsTest, FrameToRgb)
{
  auto rgbComp = std::make_shared<MockFrameComponent>();
  EXPECT_CALL(*mockFrame, getIntensityComponent()).WillOnce(Return(rgbComp));

  // Set up mock data for intensity component
  const int width = 640;
  const int height = 480;
  std::vector<uint8_t> data(width * height * 3, 255); // Mock data for RGB image

  EXPECT_CALL(*rgbComp, getWidth()).WillOnce(Return(width));
  EXPECT_CALL(*rgbComp, getDeliveredHeight()).WillOnce(Return(height));
  EXPECT_CALL(*rgbComp, getData()).WillOnce(Return(data.data()));

  cv::Mat rgbImage = frameToRgb(mockFrame);
  EXPECT_EQ(rgbImage.type(), CV_8UC3);
  EXPECT_EQ(rgbImage.cols, width);
  EXPECT_EQ(rgbImage.rows, height);
  EXPECT_TRUE(std::equal(data.begin(), data.end(), rgbImage.data));
}

TEST_F(FrameTransformationsTest, FrameToGrayscale)
{
  auto grayscaleComp = std::make_shared<MockFrameComponent>();
  EXPECT_CALL(*mockFrame, getIntensityComponent()).WillOnce(Return(grayscaleComp));

  // Set up mock data for grayscale component
  const int width = 640;
  const int height = 480;
  std::vector<uint16_t> data(width * height, 255); // Mock data for grayscale image

  EXPECT_CALL(*grayscaleComp, getWidth()).WillOnce(Return(width));
  EXPECT_CALL(*grayscaleComp, getDeliveredHeight()).WillOnce(Return(height));
  EXPECT_CALL(*grayscaleComp, getData()).WillOnce(Return(data.data()));

  cv::Mat grayscaleImage = frameToGrayscale(mockFrame);
  EXPECT_EQ(grayscaleImage.type(), CV_16UC1);
  EXPECT_EQ(grayscaleImage.cols, width);
  EXPECT_EQ(grayscaleImage.rows, height);
  EXPECT_TRUE(std::equal(data.begin(), data.end(), reinterpret_cast<uint16_t*>(grayscaleImage.data)));
}

TEST_F(FrameTransformationsTest, FrameToDepth)
{
  auto depthComp = std::make_shared<MockFrameComponent>();
  EXPECT_CALL(*mockFrame, getRangeComponent()).WillOnce(Return(depthComp));

  // Set up mock data for depth component
  const int width = 640;
  const int height = 480;
  std::vector<uint16_t> data(width * height, 255); // Mock data for depth image

  EXPECT_CALL(*depthComp, getWidth()).WillOnce(Return(width));
  EXPECT_CALL(*depthComp, getDeliveredHeight()).WillOnce(Return(height));
  EXPECT_CALL(*depthComp, getData()).WillOnce(Return(data.data()));

  cv::Mat depthImage = frameToDepth(mockFrame);
  EXPECT_EQ(depthImage.type(), CV_16UC1);
  EXPECT_EQ(depthImage.cols, width);
  EXPECT_EQ(depthImage.rows, height);
  EXPECT_TRUE(std::equal(data.begin(), data.end(), reinterpret_cast<uint16_t*>(depthImage.data)));
}

TEST_F(FrameTransformationsTest, FrameToImu)
{
  auto imuComp = std::make_shared<MockFrameComponent>();
  EXPECT_CALL(*mockFrame, getImuComponent()).WillOnce(Return(imuComp));

  // Set up mock data for IMU component
  const int numImuReadings = 5;
  std::vector<ImuData> data(numImuReadings);
  for (int i = 0; i < numImuReadings; ++i)
  {
    data[i] = {
      {1.0, 2.0, 3.0},                        // acceleration
      {4.0, 5.0, 6.0},                        // angular velocity
      {7.0, 8.0, 9.0},                        // magnetic field
      {0.1, 0.2, 0.3, 0.4},                   // orientation
      static_cast<uint64_t>(i * 1000000000UL) // timestamp
    };
  }

  EXPECT_CALL(*imuComp, getDeliveredHeight()).WillOnce(Return(numImuReadings));
  EXPECT_CALL(*imuComp, getData()).WillOnce(Return(data.data()));

  std::vector<sensor_msgs::msg::Imu> imuMessages = frameToImu(mockFrame);
  EXPECT_EQ(imuMessages.size(), numImuReadings);

  for (int i = 0; i < numImuReadings; ++i)
  {
    EXPECT_EQ(imuMessages[i].orientation.x, data[i].orientation[0]);
    EXPECT_EQ(imuMessages[i].orientation.y, data[i].orientation[1]);
    EXPECT_EQ(imuMessages[i].orientation.z, data[i].orientation[2]);
    EXPECT_EQ(imuMessages[i].orientation.w, data[i].orientation[3]);
    EXPECT_EQ(imuMessages[i].angular_velocity.x, data[i].angularVelocity[0]);
    EXPECT_EQ(imuMessages[i].angular_velocity.y, data[i].angularVelocity[1]);
    EXPECT_EQ(imuMessages[i].angular_velocity.z, data[i].angularVelocity[2]);
    EXPECT_EQ(imuMessages[i].linear_acceleration.x, data[i].acceleration[0]);
    EXPECT_EQ(imuMessages[i].linear_acceleration.y, data[i].acceleration[1]);
    EXPECT_EQ(imuMessages[i].linear_acceleration.z, data[i].acceleration[2]);
    EXPECT_EQ(imuMessages[i].header.stamp.sec, static_cast<int32_t>(data[i].timeStamp / 1000000000UL));
    EXPECT_EQ(imuMessages[i].header.stamp.nanosec, static_cast<uint32_t>(data[i].timeStamp % 1000000000UL));
  }
}
