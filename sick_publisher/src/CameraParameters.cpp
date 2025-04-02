// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include "sick_publisher/CameraParameters.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <std_msgs/msg/header.hpp>

#include "sick_publisher/CameraIntrinsics.hpp"

namespace sick
{
CameraParameters::CameraParameters(const Resolution& resolution,
                                   const BinningParameters& binningParameters,
                                   const DistortionCoefficients& distortionCoefficients,
                                   const RectificationMatrix& rectificationMatrix,
                                   const ProjectionMatrix& projectionMatrix,
                                   const RegionOfInterest& regionOfInterest,
                                   const CameraIntrinsics& cameraIntrinsics)
  : m_resolution(resolution)
  , m_binningParameters(binningParameters)
  , m_distortionCoefficients(distortionCoefficients)
  , m_rectificationMatrix(rectificationMatrix)
  , m_projectionMatrix(projectionMatrix)
  , m_regionOfInterest(regionOfInterest)
  , m_cameraIntrinsics(cameraIntrinsics)
{
}

CameraParameters::operator sensor_msgs::msg::CameraInfo::SharedPtr() const
{
  auto msg = std::make_shared<sensor_msgs::msg::CameraInfo>();

  // Set the header
  rclcpp::Clock clock;
  msg->header.stamp = clock.now();
  msg->header.frame_id = "camera_frame";

  // Set the resolution
  msg->height = m_resolution.height;
  msg->width = m_resolution.width;

  // Set the binning parameters
  msg->binning_x = m_binningParameters.binningHorizontal;
  msg->binning_y = m_binningParameters.binningVertical;

  // Set the distortion model and coefficients
  msg->distortion_model = "plumb_bob";
  msg->d = {m_distortionCoefficients.k1,
            m_distortionCoefficients.k2,
            m_distortionCoefficients.k3,
            m_distortionCoefficients.p1,
            m_distortionCoefficients.p2};

  // Set the intrinsic camera matrix (K)
  msg->k = {m_cameraIntrinsics.focalLength,
            0.0,
            m_cameraIntrinsics.principalPointU,
            0.0,
            m_cameraIntrinsics.focalLength * m_cameraIntrinsics.aspectRatio,
            m_cameraIntrinsics.principalPointV,
            0.0,
            0.0,
            1.0};

  // Set the rectification matrix (R)
  msg->r = {m_rectificationMatrix.r11,
            m_rectificationMatrix.r12,
            m_rectificationMatrix.r13,
            m_rectificationMatrix.r21,
            m_rectificationMatrix.r22,
            m_rectificationMatrix.r23,
            m_rectificationMatrix.r31,
            m_rectificationMatrix.r32,
            m_rectificationMatrix.r33};

  // Set the projection matrix (P)
  msg->p = {m_projectionMatrix.fx_prime,
            0.0,
            m_projectionMatrix.cx_prime,
            m_projectionMatrix.Tx,
            0.0,
            m_projectionMatrix.fy_prime,
            m_projectionMatrix.cy_prime,
            m_projectionMatrix.Ty,
            0.0,
            0.0,
            1.0,
            0.0};

  // Set the region of interest (ROI)
  msg->roi.x_offset = m_regionOfInterest.roiXOffset;
  msg->roi.y_offset = m_regionOfInterest.roiYOffset;
  msg->roi.height = m_regionOfInterest.roiHeight;
  msg->roi.width = m_regionOfInterest.roiWidth;
  msg->roi.do_rectify = m_regionOfInterest.roiDoRectify;

  return msg;
}

CameraIntrinsics CameraParameters::getCameraIntrinsics() const
{
  return m_cameraIntrinsics;
}

} // namespace sick
