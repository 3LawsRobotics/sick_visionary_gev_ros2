// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_INCLUDE_SICK_PUBLISHER_CAMERAPARAMETERS_HPP
#define SICK_PUBLISHER_INCLUDE_SICK_PUBLISHER_CAMERAPARAMETERS_HPP

#include <sensor_msgs/msg/camera_info.hpp>

#include "CameraIntrinsics.hpp"

namespace sick
{

/// @brief Resolution struct.
struct Resolution
{
  float height;
  float width;
};

/// @brief BinningParameters struct.
struct BinningParameters
{
  int binningHorizontal;
  int binningVertical;
};

/// @brief DistortionCoefficients struct.
struct DistortionCoefficients
{
  float k1;
  float k2;
  float k3;
  float p1;
  float p2;
};

/// @brief RectificationMatrix struct.
struct RectificationMatrix
{
  float r11;
  float r12;
  float r13;
  float r21;
  float r22;
  float r23;
  float r31;
  float r32;
  float r33;
};

/// @brief ProjectionMatrix struct.
struct ProjectionMatrix
{
  float fx_prime;
  float fy_prime;
  float cx_prime;
  float cy_prime;
  float Tx;
  float Ty;
};

/// @brief RegionOfInterest struct.
struct RegionOfInterest
{
  int roiXOffset;
  int roiYOffset;
  int roiHeight;
  int roiWidth;
  bool roiDoRectify;
};

/// @brief CameraParameters class.
/// @details This class stores camera parameters relevant for the camera info message.
class CameraParameters
{
public:
  CameraParameters() = default;
  CameraParameters(const Resolution& resolution,
                   const BinningParameters& binningParameters,
                   const DistortionCoefficients& distortionCoefficients,
                   const RectificationMatrix& rectificationMatrix,
                   const ProjectionMatrix& projectionMatrix,
                   const RegionOfInterest& regionOfInterest,
                   const CameraIntrinsics& cameraIntrinsics);

  operator sensor_msgs::msg::CameraInfo::SharedPtr() const;

  CameraIntrinsics getCameraIntrinsics() const;

private:
  Resolution m_resolution;
  BinningParameters m_binningParameters;
  DistortionCoefficients m_distortionCoefficients;
  RectificationMatrix m_rectificationMatrix;
  ProjectionMatrix m_projectionMatrix;
  RegionOfInterest m_regionOfInterest;
  CameraIntrinsics m_cameraIntrinsics;
};

} // namespace sick

#endif // CAMERA_PARAMETERS_HPP_