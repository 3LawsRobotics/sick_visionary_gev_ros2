// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_ADAPTER_GENISTREAMSDK_CAMERACONTROL_HPP
#define SICK_PUBLISHER_ADAPTER_GENISTREAMSDK_CAMERACONTROL_HPP

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <base/ErrorCode.hpp>
#include <sick_publisher/ICameraControl.hpp>

namespace genistream
{
class FrameGrabber;
class ICamera;
class IAnyParameters;

} // namespace genistream

namespace sick
{
class ICameraFrame;

class CameraControl : public ICameraControl
{
public:
  /// @brief Constructor of CameraDiscovery
  /// @param[in] serial Serial number of the device to control
  /// @param[in] camPtr Pointer to genistream::ICamera
  CameraControl(const std::string& serial, std::shared_ptr<genistream::ICamera> camPtr);

  ErrorCode connect() override;
  ErrorCode disableFilters() override;
  ErrorCode loadConfig(const std::vector<std::string>& gevComponents,
                       const std::vector<std::pair<std::string, std::string>>& gevParams) override;
  ErrorCode disconnect() override;
  ErrorCode startStreaming() override;
  ErrorCode stopStreaming() override;
  FrameOrError<std::shared_ptr<ICameraFrame>> getNextFrame() override;
  ParameterOrError<std::string> getCameraParameter(const std::string& paramName) override;
  ParameterOrError<std::string> getCameraParameter(const std::string& featureName,
                                                   const std::string& paramName) override;
  ParameterOrError<float> getCameraParameterFloat(const std::string& featureName,
                                                  const std::string& paramName) override;
  bool isComponentActive(const std::string& compName) override;
  std::string getComponentPixelFormat(const std::string& component) override;

private:
  std::string m_serial{};
  std::shared_ptr<genistream::ICamera> m_iCamera{};
  std::shared_ptr<genistream::FrameGrabber> m_grabber{};
  std::shared_ptr<genistream::IAnyParameters> m_anyParameters{};
};
} // namespace sick

#endif // GENISTREAM_CAMERA_DISCOVERY_HPP_