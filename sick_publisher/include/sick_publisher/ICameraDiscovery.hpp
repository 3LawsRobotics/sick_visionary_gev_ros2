// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_INCLUDE_SICK_PUBLISHER_ICAMERADISCOVERY_HPP
#define SICK_PUBLISHER_INCLUDE_SICK_PUBLISHER_ICAMERADISCOVERY_HPP

#include <memory>
#include <string>
#include <vector>

#include <base/Compiler.hpp>
#include <base/ErrorCode.hpp>
#include <base/ErrorTrace.hpp>
#include <base/Result.hpp>

#include "CameraInfo.hpp"
#include "ICameraControl.hpp"

namespace sick
{
template <typename T>
using CameraOrError = Result<T, ErrorTrace>;

class ICameraDiscovery
{
public:
  virtual ~ICameraDiscovery() = default;

  /// @brief Discover all cameras in a network
  /// @return Discovery status with error code
  virtual ErrorCode discoverCameras() = 0;

  /// @brief Get camera info for all discovery cameras in a network
  /// @return camera information
  SICK_NO_DISCARD virtual std::vector<CameraInfo> getDiscoveredCameras() = 0;

  /// @brief Get camera control interface for a device with a specific serial number
  /// @param[in] serial A device serial number
  /// @return Shared pointers to a camera control interface
  virtual CameraOrError<std::shared_ptr<ICameraControl>> getCameraControl(const std::string& serial) = 0;
};
} // namespace sick

#endif // ICAMERA_DISCOVERY_HPP_
