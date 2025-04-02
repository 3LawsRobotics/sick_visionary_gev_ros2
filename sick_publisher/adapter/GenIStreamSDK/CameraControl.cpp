// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include "CameraControl.hpp"

#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

#include <genistream/Exceptions.h>
#include <genistream/FrameGrabber.h>
#include <genistream/GrabResult.h>
#include <genistream/IAnyParameters.h>
#include <genistream/ICamera.h>

#include "GenIStreamSDK/CameraFrame.hpp"
#include "base/ErrorCode.hpp"
#include "base/Logging.hpp"
#include "sick_publisher/ICameraControl.hpp"

namespace sick
{
class ICameraFrame;

CameraControl::CameraControl(const std::string& serial, std::shared_ptr<genistream::ICamera> camPtr)
  : m_serial(serial)
  , m_iCamera(camPtr)
  , m_anyParameters(m_iCamera->getAnyParameters())
{
}

ErrorCode CameraControl::connect()
{
  if (!m_iCamera->isConnected())
  {
    errLog() << "Could not connect to camera: " << m_serial << "\n";
    return ErrorCode::ConnectionFailed;
  }
  return ErrorCode::OK;
}

ErrorCode CameraControl::disableFilters()
{
  // Ensure camera and configuration are available
  if (!m_iCamera->isConnected())
  {
    errLog(m_serial) << "Camera lost connection";
    return ErrorCode::ConnectionFailed;
  }

  // Stop streaming before configuring
  if (stopStreaming() != ErrorCode::OK)
  {
    errLog(m_serial) << "Unable to stop streaming before disabeling parameters";
    return ErrorCode::StopStreamFailed;
  }
  auto filterIds = m_anyParameters->getEntryIdsForEnum("Scan3dDataFilterSelector");
  for (auto filterID : filterIds)
  {
    if (m_anyParameters->isAccessible(filterID))
    {
      m_anyParameters->setValueFromString("Scan3dDataFilterSelector",
                                          m_anyParameters->getEnumEntrySymbolicName(filterID));
      m_anyParameters->setValueFromString("Scan3dDataFilterEnable", "0");
    }
  }
  return ErrorCode::OK;
}

ErrorCode CameraControl::loadConfig(const std::vector<std::string>& gevComponents,
                                    const std::vector<std::pair<std::string, std::string>>& gevParams)
{
  // Ensure camera and configuration are available
  if (!m_iCamera->isConnected())
  {
    errLog() << "Unable to load parameters: " << m_serial << "\n";
    return ErrorCode::ConnectionFailed;
  }

  // Stop streaming before configuring
  if (stopStreaming() != ErrorCode::OK)
  {
    errLog() << "Unable to stop streaming before loading parameters: " << m_serial << "\n";
    return ErrorCode::StopStreamFailed;
  }

  // Get accessible components
  auto ids = m_anyParameters->getEntryIdsForEnum("ComponentSelector");
  std::vector<genistream::EnumEntrySymbolicName> accessibleComponents;
  for (auto id : ids)
  {
    if (m_anyParameters->isAccessible(id))
    {
      accessibleComponents.push_back(m_anyParameters->getEnumEntrySymbolicName(id).c_str());
    }
  }

  // Enable components in gevComponents
  for (const auto& component : gevComponents)
  {
    m_anyParameters->setValueFromString("ComponentSelector", component);
    m_anyParameters->setValueFromString("ComponentEnable", "true");
  }

  // Disable components not in gevComponents but in accessibleComponents
  for (const auto& accessibleComp : accessibleComponents)
  {
    if (std::find(gevComponents.begin(), gevComponents.end(), accessibleComp) == gevComponents.end())
    {
      m_anyParameters->setValueFromString("ComponentSelector", accessibleComp);
      m_anyParameters->setValueFromString("ComponentEnable", "false");
    }
  }

  // Set up other parameters from gevParams
  for (const auto& [key, value] : gevParams)
  {
    m_anyParameters->setValueFromString(key, value);
  }

  return ErrorCode::OK;
}

ErrorCode CameraControl::disconnect()
{
  m_iCamera->disconnect();
  return ErrorCode::OK;
}

ErrorCode CameraControl::startStreaming()
{
  if (!m_grabber)
  {
    const std::size_t kBufferSize{5};
    m_grabber = m_iCamera->createFrameGrabber(kBufferSize);
  }
  m_grabber->start();
  return ErrorCode::OK;
}

ErrorCode CameraControl::stopStreaming()
{
  if (m_grabber)
  {
    m_grabber->stop();
  }
  return ErrorCode::OK;
}

FrameOrError<std::shared_ptr<ICameraFrame>> CameraControl::getNextFrame()
{
  if (!m_grabber->isCameraStarted())
  {
    return SICK_FRAME_GRABBER_FAILED("Camera stream is not started. Start acquisition first to receive frames");
  }
  genistream::GrabResult result = m_grabber->grabNext();
  if (result.hasFrame())
  {
    return std::static_pointer_cast<ICameraFrame>(std::make_shared<CameraFrame>(result.getOrThrow(false)));
  }
  return SICK_FRAME_GRABBER_FAILED("No frame received. Check if MTU is set to 9000 for the used network interface.");
}

ParameterOrError<std::string> CameraControl::getCameraParameter(const std::string& paramName)
{
  if (!m_iCamera->isConnected())
  {
    return SICK_CONNECTION_FAILED("Connect to camera before requesting parameter.");
  }
  try
  {
    std::shared_ptr<genistream::IAnyParameters> anyParameters{m_iCamera->getAnyParameters()};
    std::string value = anyParameters->getValueAsString(paramName);
    return value;
  }
  catch (const genistream::DisconnectedException&)
  {
    return SICK_CONNECTION_FAILED("Connect to camera before requesting parameter.");
  }
  catch (const genistream::ParameterAccessException& ex)
  {
    return SICK_PARAM_FAILED("Parameter is not accessible or doesn't exist:" + std::string(ex.what()));
  }
}

ParameterOrError<std::string> CameraControl::getCameraParameter(const std::string& featureName,
                                                                const std::string& paramName)
{
  if (!m_iCamera->isConnected())
  {
    return SICK_CONNECTION_FAILED("Connect to camera before requesting parameter.");
  }
  try
  {
    m_anyParameters->setEnum(featureName + "Selector", paramName);
    auto value = m_anyParameters->getValueAsString(featureName + "Value");
    return value;
  }
  catch (const genistream::DisconnectedException&)
  {
    return SICK_CONNECTION_FAILED("Connect to camera before requesting parameter.");
  }
  catch (const genistream::ParameterAccessException&)
  {
    return SICK_PARAM_FAILED("Parameter is not accessible or doesn't exist.");
  }
}

ParameterOrError<float> CameraControl::getCameraParameterFloat(const std::string& featureName,
                                                               const std::string& paramName)
{
  if (!m_iCamera->isConnected())
  {
    return SICK_CONNECTION_FAILED("Connect to camera before requesting parameter.");
  }
  try
  {
    m_anyParameters->setEnum(featureName + "Selector", paramName);
    auto value = m_anyParameters->getFloat(featureName + "Value");
    return value;
  }
  catch (const genistream::DisconnectedException&)
  {
    return SICK_CONNECTION_FAILED("Connect to camera before requesting parameter.");
  }
  catch (const genistream::ParameterAccessException&)
  {
    return SICK_PARAM_FAILED("Parameter is not accessible or doesn't exist.");
  }
}

bool CameraControl::isComponentActive(const std::string& compName)
{
  m_anyParameters->setEnum("ComponentSelector", compName);
  return m_anyParameters->getBool("ComponentEnable");
};

std::string CameraControl::getComponentPixelFormat(const std::string& component)
{
  m_anyParameters->setEnum("ComponentSelector", component);
  return m_anyParameters->getEnum("PixelFormat");
}

} // namespace sick
