// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include <memory> // for __shared_ptr_access

#include <gtest/gtest.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>

#include "sick_publisher/CameraIntrinsics.hpp"
#include "sick_publisher/CameraParameters.hpp"

using namespace sick;

class CameraInfoParametersTest : public ::testing::Test
{
protected:
  CameraParameters cameraParameters;
};

TEST_F(CameraInfoParametersTest, ParameterConstructor)
{
  // Set up parameters
  Resolution resolution = {480, 640};
  BinningParameters binningParameters = {2, 2};
  DistortionCoefficients distortionCoefficients = {0.1, 0.2, 0.3, 0.4, 0.5};
  RectificationMatrix rectificationMatrix = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  ProjectionMatrix projectionMatrix = {500, 500, 320, 240, 0, 0};
  RegionOfInterest regionOfInterest = {10, 20, 100, 200, true};
  CameraIntrinsics cameraIntrinsics{};
  cameraIntrinsics.focalLength = 50.;
  cameraIntrinsics.principalPointU = 320.;
  cameraIntrinsics.principalPointV = 240.;
  cameraIntrinsics.aspectRatio = 1.;
  cameraIntrinsics.scaleC = 1.;
  cameraIntrinsics.offset = 0.;

  // Create CameraParameters object
  CameraParameters cameraParameters(resolution,
                                    binningParameters,
                                    distortionCoefficients,
                                    rectificationMatrix,
                                    projectionMatrix,
                                    regionOfInterest,
                                    cameraIntrinsics);

  // Check values
  EXPECT_EQ(cameraParameters.getCameraIntrinsics().focalLength, cameraIntrinsics.focalLength);
  EXPECT_EQ(cameraParameters.getCameraIntrinsics().principalPointU, cameraIntrinsics.principalPointU);
  EXPECT_EQ(cameraParameters.getCameraIntrinsics().principalPointV, cameraIntrinsics.principalPointV);
  EXPECT_EQ(cameraParameters.getCameraIntrinsics().aspectRatio, cameraIntrinsics.aspectRatio);
  EXPECT_EQ(cameraParameters.getCameraIntrinsics().scaleC, cameraIntrinsics.scaleC);
  EXPECT_EQ(cameraParameters.getCameraIntrinsics().offset, cameraIntrinsics.offset);
}

TEST_F(CameraInfoParametersTest, OverloadedOperator)
{
  // Set up parameters
  Resolution resolution = {480, 640};
  BinningParameters binningParameters = {2, 2};
  DistortionCoefficients distortionCoefficients = {0.1, 0.2, 0.3, 0.4, 0.5};
  RectificationMatrix rectificationMatrix = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  ProjectionMatrix projectionMatrix = {500, 500, 320, 240, 0, 0};
  RegionOfInterest regionOfInterest = {10, 20, 100, 200, true};
  CameraIntrinsics cameraIntrinsics{};
  cameraIntrinsics.focalLength = 50.;
  cameraIntrinsics.principalPointU = 320.;
  cameraIntrinsics.principalPointV = 240.;
  cameraIntrinsics.aspectRatio = 1.;
  cameraIntrinsics.scaleC = 1.;
  cameraIntrinsics.offset = 0.;

  // Create CameraParameters object
  CameraParameters cameraParameters(resolution,
                                    binningParameters,
                                    distortionCoefficients,
                                    rectificationMatrix,
                                    projectionMatrix,
                                    regionOfInterest,
                                    cameraIntrinsics);

  // Use overloaded operator
  auto cameraInfo = static_cast<sensor_msgs::msg::CameraInfo::SharedPtr>(cameraParameters);

  // Check values
  EXPECT_EQ(cameraInfo->height, resolution.height);
  EXPECT_EQ(cameraInfo->width, resolution.width);
  EXPECT_EQ(cameraInfo->binning_x, binningParameters.binningHorizontal);
  EXPECT_EQ(cameraInfo->binning_y, binningParameters.binningVertical);
  EXPECT_EQ(cameraInfo->distortion_model, "plumb_bob");
  EXPECT_EQ(cameraInfo->d[0], distortionCoefficients.k1);
  EXPECT_EQ(cameraInfo->d[1], distortionCoefficients.k2);
  EXPECT_EQ(cameraInfo->d[2], distortionCoefficients.k3);
  EXPECT_EQ(cameraInfo->d[3], distortionCoefficients.p1);
  EXPECT_EQ(cameraInfo->d[4], distortionCoefficients.p2);
  EXPECT_EQ(cameraInfo->k[0], cameraIntrinsics.focalLength);
  EXPECT_EQ(cameraInfo->k[4], cameraIntrinsics.focalLength * cameraIntrinsics.aspectRatio);
  EXPECT_EQ(cameraInfo->k[2], cameraIntrinsics.principalPointU);
  EXPECT_EQ(cameraInfo->k[5], cameraIntrinsics.principalPointV);
  EXPECT_EQ(cameraInfo->r[0], rectificationMatrix.r11);
  EXPECT_EQ(cameraInfo->r[1], rectificationMatrix.r12);
  EXPECT_EQ(cameraInfo->r[2], rectificationMatrix.r13);
  EXPECT_EQ(cameraInfo->r[3], rectificationMatrix.r21);
  EXPECT_EQ(cameraInfo->r[4], rectificationMatrix.r22);
  EXPECT_EQ(cameraInfo->r[5], rectificationMatrix.r23);
  EXPECT_EQ(cameraInfo->r[6], rectificationMatrix.r31);
  EXPECT_EQ(cameraInfo->r[7], rectificationMatrix.r32);
  EXPECT_EQ(cameraInfo->r[8], rectificationMatrix.r33);
  EXPECT_EQ(cameraInfo->p[0], projectionMatrix.fx_prime);
  EXPECT_EQ(cameraInfo->p[5], projectionMatrix.fy_prime);
  EXPECT_EQ(cameraInfo->p[2], projectionMatrix.cx_prime);
  EXPECT_EQ(cameraInfo->p[6], projectionMatrix.cy_prime);
  EXPECT_EQ(cameraInfo->p[3], projectionMatrix.Tx);
  EXPECT_EQ(cameraInfo->p[7], projectionMatrix.Ty);
  EXPECT_EQ(cameraInfo->roi.x_offset, regionOfInterest.roiXOffset);
  EXPECT_EQ(cameraInfo->roi.y_offset, regionOfInterest.roiYOffset);
  EXPECT_EQ(cameraInfo->roi.height, regionOfInterest.roiHeight);
  EXPECT_EQ(cameraInfo->roi.width, regionOfInterest.roiWidth);
  EXPECT_EQ(cameraInfo->roi.do_rectify, regionOfInterest.roiDoRectify);
}
