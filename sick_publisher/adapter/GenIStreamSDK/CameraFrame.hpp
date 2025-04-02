// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_ADAPTER_GENISTREAMSDK_CAMERAFRAME_HPP
#define SICK_PUBLISHER_ADAPTER_GENISTREAMSDK_CAMERAFRAME_HPP

#include <cstdint>
#include <memory>
#include <string>

#include <sick_publisher/ICameraFrame.hpp>

namespace genistream
{
namespace frame
{
class IComponent;
class IFrame;
} // namespace frame
} // namespace genistream

namespace sick
{

class FrameComponent : public IFrameComponent
{
public:
  FrameComponent(std::shared_ptr<const genistream::frame::IComponent> frameComponent);
  int getWidth() const override;
  int getDeliveredHeight() const override;
  void* getData() const override;

private:
  std::shared_ptr<const genistream::frame::IComponent> m_component{nullptr};
};

class CameraFrame : public ICameraFrame
{
public:
  CameraFrame(const std::shared_ptr<genistream::frame::IFrame> frame);
  /// Access functions for raw frame data for different modalities
  std::shared_ptr<IFrameComponent> getRangeComponent() const override;
  std::shared_ptr<IFrameComponent> getIntensityComponent() const override;
  std::shared_ptr<IFrameComponent> getDepthComponent() const override;
  std::shared_ptr<IFrameComponent> getImuComponent() const override;

  std::string getFrameID() const override;
  uint64_t getTimestamp() const override;

private:
  std::shared_ptr<genistream::frame::IFrame> m_frame{nullptr};
};

} // namespace sick

#endif // GENISTREAM_CAMERA_FRAME_HPP_