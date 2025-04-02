// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include "CameraDiscovery.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <filesystem>
#include <mutex>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <base/ErrorCode.hpp>
#include <genistream/CameraDiscovery.h>
#include <genistream/DiscoveredCamera.h>
#include <genistream/Ip4Address.h>
#include <sick_publisher/CameraInfo.hpp>
#include <sick_publisher/ICameraDiscovery.hpp>

#include "CameraControl.hpp"
#include "base/Logging.hpp"
#include "base/Platform.hpp"

namespace sick
{
class ICameraControl;

static std::mutex discoveryMutex;

CameraDiscovery::CameraDiscovery()
{
  namespace fs = std::filesystem;
  // build cti file relative path
  fs::path platform = getPlatformString();
  fs::path ctiFilePath{};
  if (!platform.empty())
  {
    ctiFilePath = fs::path{"cti"} / platform / fs::path{kSICKCTI};
  }

  // build cti file absolute path
  const std::string currentPath = ament_index_cpp::get_package_share_directory("sick_visionary_gev_ros2") +
                                  std::filesystem::path::preferred_separator + ctiFilePath.string();
  m_cameraDiscovery = genistream::CameraDiscovery::createFromProducerFile(currentPath);
}

ErrorCode CameraDiscovery::discoverCameras()
{
  std::lock_guard<std::mutex> lock(discoveryMutex);

  if (!m_cameraDiscovery)
  {
    return ErrorCode::ConnectionFailed;
  }

  const auto timeout = std::chrono::milliseconds(1000);
  try
  {
    m_cameras = m_cameraDiscovery->scanForCameras(timeout);
  }
  catch (const std::exception& e)
  {
    errLog("CameraDiscovery") << e.what();
  }

  infoLog("CameraDiscovery") << m_cameras.size() << " GigE device(s) discovered";
  return m_cameras.empty() ? ErrorCode::NoData : ErrorCode::OK;
}

std::vector<CameraInfo> CameraDiscovery::getDiscoveredCameras()
{
  std::lock_guard<std::mutex> lock(discoveryMutex);

  std::vector<CameraInfo> info{};
  info.reserve(m_cameras.size());

  for (auto cam : m_cameras)
  {
    info.emplace_back(cam->getId(), cam->getModel(), cam->getIpAddress().toString(), cam->getSerialNumber());
  }

  return info;
}

CameraOrError<std::shared_ptr<ICameraControl>> CameraDiscovery::getCameraControl(const std::string& serial)
{
  std::lock_guard<std::mutex> lock(discoveryMutex);

  using Status = genistream::DiscoveredCamera::AccessStatus;

  std::shared_ptr<genistream::DiscoveredCamera> cam{nullptr};
  const int kMaxRetries = 3;
  int retries{0};
  bool foundMatch{false};

  while (!foundMatch && retries++ < kMaxRetries)
  {
    for (auto camPtr : m_cameras)
    {
      if (camPtr->getSerialNumber() == serial)
      {
        if (camPtr->getAccessStatus() != Status::NO_ACCESS)
        {
          // found match
          cam = camPtr;
          foundMatch = true;
        }
        else
        {
          //! cf. TutorialGenIStream.cpp
          //! Before trying to open, we can check the access status of the (first discovered) device.
          //! If the status is "NOACCESS", most likely a wrong IP address (not matching the subnet of the network card
          //! it is connected to).
          //! We can demonstrate how a suitable IP address can be "forced" into the device, however, configuring
          //! the device to boot into expected IP subnet is recommended instead.
          auto proposedIp = m_cameraDiscovery->proposeIp(camPtr);
          const auto timeout = std::chrono::milliseconds(1000 + retries * 1000);
          m_cameraDiscovery->forceIp(camPtr, proposedIp.getIpAddress(), proposedIp.getSubnetMask(), timeout);

          // update m_cameras after a newly proposed ip
          m_cameras = m_cameraDiscovery->scanForCameras(timeout);
        }

        break;
      }
    }
  }

  if (!cam)
  {
    return SICK_CONNECTION_FAILED("Camera with serial number: " + serial + " is not available!");
  }
  //! cf. TutorialGenIStream.cpp
  //! If the access status is still not "READWRITE" (meaning it is ready to open) or if we failed for any reason to
  //! rediscover the camera, stop the example. This can happen because of multiple reasons, including that the camera
  //! is already open by another application
  if (cam->getAccessStatus() != Status::READ_WRITE)
  {
    return SICK_CONNECTION_FAILED("Camera access is not in read/write mode!");
  }
  auto iCamera = m_cameraDiscovery->connectTo(cam);
  return std::static_pointer_cast<ICameraControl>(std::make_shared<CameraControl>(serial, iCamera));
}
} // namespace sick
