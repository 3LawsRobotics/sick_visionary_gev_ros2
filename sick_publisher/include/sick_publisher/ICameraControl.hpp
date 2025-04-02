// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_INCLUDE_SICK_PUBLISHER_ICAMERACONTROL_HPP
#define SICK_PUBLISHER_INCLUDE_SICK_PUBLISHER_ICAMERACONTROL_HPP

#include <memory>
#include <utility>

#include <base/ErrorCode.hpp>
#include <base/ErrorTrace.hpp>
#include <base/Result.hpp>

#include "sick_publisher/ICameraFrame.hpp"

namespace sick
{

template <typename T>
using FrameOrError = Result<T, ErrorTrace>;

template <typename T>
using ParameterOrError = Result<T, ErrorTrace>;

/// @brief Interface for basic camera operations
class ICameraControl
{
public:
  virtual ~ICameraControl() {}

  /// @brief Get connection to the device
  /// @return ErrorCode of the status
  virtual ErrorCode connect() = 0;

  /// @brief Disables all accessible filters
  /// @return ErrorCode of the status
  virtual ErrorCode disableFilters() = 0;

  /// @brief Loads the yaml file parameters
  /// @return ErrorCode of the status
  virtual ErrorCode loadConfig(const std::vector<std::string>& gevComponents,
                               const std::vector<std::pair<std::string, std::string>>& gevParams) = 0;

  /// @brief Disconnect to the device
  /// @return ErrorCode of the status
  virtual ErrorCode disconnect() = 0;

  /// @brief Start device streaming
  /// @return ErrorCode of the status
  virtual ErrorCode startStreaming() = 0;

  /// @brief Stop device streaming
  /// @return ErrorCode of the status
  virtual ErrorCode stopStreaming() = 0;

  /// @brief Get next frame from frame grabber
  /// @return Shared pointer to a frame interface or ErrorCode
  virtual FrameOrError<std::shared_ptr<ICameraFrame>> getNextFrame() = 0;

  /// @brief Get parameter from camera
  /// @return Parameter value as string or ErrorCode
  virtual ParameterOrError<std::string> getCameraParameter(const std::string& paramName) = 0;

  /// @brief Get parameter from camera
  /// @param featureName Name of the feature.
  /// @param paramName Name of the parameter.
  /// @return Parameter value as string or ErrorCode
  virtual ParameterOrError<std::string> getCameraParameter(const std::string& featureName,
                                                           const std::string& paramName) = 0;

  /// @brief Get parameter from camera
  /// @param featureName Name of the feature.
  /// @param paramName Name of the parameter.
  /// @return Parameter value as float or ErrorCode
  virtual ParameterOrError<float> getCameraParameterFloat(const std::string& featureName,
                                                          const std::string& paramName) = 0;

  /// @brief Check if component is activated through the config file.
  /// @param compName Name of the component.
  /// @return true if the component has been enabled.
  virtual bool isComponentActive(const std::string& compName) = 0;

  /// @brief Retrieves the pixel format of a specified component.
  /// @param component The name of the component whose pixel format is to be retrieved.
  /// @return A string representing the pixel format of the specified component.
  virtual std::string getComponentPixelFormat(const std::string& component) = 0;
};
} // namespace sick

#endif // ICAMERA_CONTROL_HPP_