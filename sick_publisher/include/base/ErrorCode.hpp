// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_INCLUDE_BASE_ERRORCODE_HPP
#define SICK_PUBLISHER_INCLUDE_BASE_ERRORCODE_HPP

#include <string>

namespace sick
{
/// @brief The ErrorCode enum defines error states for camera operations#
enum class ErrorCode : uint32_t
{
  OK = 0,                   ///< Operation successful
  ConnectionFailed,         ///< Failed to connect to camera
  DisconnectionFailed,      ///< Failed to disconnect from the camera
  StartStreamFailed,        ///< Failed to start the camera stream
  StopStreamFailed,         ///< Failed to stop the camera stream
  UnsupportedConfiguration, ///< The requested configuration is not supported
  InvalidParameter,         ///< One or more parameters are invalid
  DeviceNotFound,           ///< The specified camera device could not be found
  DeviceBusy,               ///< The camera device is already in use
  Timeout,                  ///< Operation timed out
  InternalError,            ///< An internal error occured
  NotConnected,             ///< Attempted an operation without being connected
  NotInitialized,           ///< The camera has not been properly initialized
  NoData,                   ///< No data available to return
};

#define SICK_MAKE_ERROR(TYPE, MESSAGE) ErrorTrace::create(TYPE, MESSAGE, __FILE__, __func__, __LINE__)
#define SICK_CONNECTION_FAILED(MESSAGE) SICK_MAKE_ERROR(ErrorCode::ConnectionFailed, MESSAGE)
#define SICK_FRAME_GRABBER_FAILED(MESSAGE) SICK_MAKE_ERROR(ErrorCode::NoData, MESSAGE)
#define SICK_INTERNAL_ERROR(MESSAGE) SICK_MAKE_ERROR(ErrorCode::InternalError, MESSAGE)
#define SICK_PARAM_FAILED(MESSAGE) SICK_MAKE_ERROR(ErrorCode::InvalidParameter, MESSAGE)

} // namespace sick

#endif // SICK_PUBLISHER_INCLUDE_BASE_ERRORCODE_HPP