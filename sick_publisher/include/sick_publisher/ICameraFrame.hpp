// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_INCLUDE_SICK_PUBLISHER_ICAMERAFRAME_HPP
#define SICK_PUBLISHER_INCLUDE_SICK_PUBLISHER_ICAMERAFRAME_HPP

#include <cstdint>
#include <memory>
#include <string>

#include <base/Compiler.hpp>
#ifdef ROS2_HUMBLE
  #include <cv_bridge/cv_bridge.h>
#else
  #include <cv_bridge/cv_bridge.hpp>
#endif
#include <sick_publisher/CameraIntrinsics.hpp>

namespace sick
{
class IFrameComponent
{
public:
  virtual ~IFrameComponent() {}

  SICK_NO_DISCARD virtual int getWidth() const = 0;
  SICK_NO_DISCARD virtual int getDeliveredHeight() const = 0;
  SICK_NO_DISCARD virtual void* getData() const = 0;
};

class ICameraFrame
{
public:
  virtual ~ICameraFrame() {}

  /// Access functions for raw frame data for different modalities
  SICK_NO_DISCARD virtual std::shared_ptr<IFrameComponent> getRangeComponent() const = 0;
  SICK_NO_DISCARD virtual std::shared_ptr<IFrameComponent> getIntensityComponent() const = 0;
  SICK_NO_DISCARD virtual std::shared_ptr<IFrameComponent> getDepthComponent() const = 0;
  SICK_NO_DISCARD virtual std::shared_ptr<IFrameComponent> getImuComponent() const = 0;

  SICK_NO_DISCARD virtual std::string getFrameID() const = 0;
  SICK_NO_DISCARD virtual uint64_t getTimestamp() const = 0;
};
} // namespace sick

#endif // ICAMERA_FRAME_HPP_
