// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_ADAPTER_GENISTREAMSDK_CAMERADISCOVERY_HPP
#define SICK_PUBLISHER_ADAPTER_GENISTREAMSDK_CAMERADISCOVERY_HPP

#include <memory>
#include <string>
#include <vector>

#include <sick_publisher/ICameraDiscovery.hpp>

#include "base/ErrorCode.hpp"

namespace genistream
{
class CameraDiscovery;
class DiscoveredCamera;
} // namespace genistream

namespace sick
{
class ICameraControl;
struct CameraInfo;

class CameraDiscovery : public ICameraDiscovery
{
public:
  static CameraDiscovery& getInstance()
  {
    static CameraDiscovery instance;
    return instance;
  }

  /// @brief Scans all network interfaces and returns information on found cameras.
  ErrorCode discoverCameras() override;

  /// @brief Get camera info for all discovery cameras in a network
  /// @return vector with camera information
  std::vector<CameraInfo> getDiscoveredCameras() override;

  /// @brief Get camera control interface from a device with a specific serial number
  /// @param[in] serial A device serial number
  /// @return Shared pointer to a camera control interface or an @ref ErrorTrace in case of error
  CameraOrError<std::shared_ptr<ICameraControl>> getCameraControl(const std::string& serial) override;

private:
  /// @brief Constructor of CameraDiscovery
  CameraDiscovery();

  CameraDiscovery(const CameraDiscovery&) = delete;
  CameraDiscovery& operator=(const CameraDiscovery&) = delete;

  /// Path to cti file
  std::string m_ctiFilePath{};
  /// cti file name as constant
  static constexpr char kSICKCTI[] = "SICKGigEVisionTL.cti";

  std::shared_ptr<genistream::CameraDiscovery> m_cameraDiscovery{nullptr};

  /// cameras to be controlled
  /// Set intersection between the devices in the network and the input configuration
  std::vector<std::shared_ptr<genistream::DiscoveredCamera>> m_cameras{};
};
} // namespace sick

#endif // GENISTREAM_CAMERA_DISCOVERY_HPP_
