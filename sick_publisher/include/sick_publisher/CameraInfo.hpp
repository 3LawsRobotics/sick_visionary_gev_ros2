// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_INCLUDE_SICK_PUBLISHER_CAMERAINFO_HPP
#define SICK_PUBLISHER_INCLUDE_SICK_PUBLISHER_CAMERAINFO_HPP

#include <string>

namespace sick
{
/// @brief Data class for camera info
struct CameraInfo
{
  std::string m_id;
  std::string m_model;
  std::string m_ipAddress;
  std::string m_serialNumber;
  bool m_isConnected;

  CameraInfo(const std::string& id,
             const std::string& model,
             const std::string& ipAddress,
             const std::string& serialNumber);
};
} // namespace sick

#endif // CAMERA_INFO_HPP_