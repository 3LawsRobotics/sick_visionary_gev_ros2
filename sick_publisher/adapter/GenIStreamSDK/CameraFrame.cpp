// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense
#include "CameraFrame.hpp"

#include <cstdint>
#include <string>

#include <genistream/CameraParameters.h>
#include <genistream/frame/IComponent.h>
#include <genistream/frame/IFrame.h>
#include <gentlcpp/IBuffer.h>

namespace sick
{
class IFrameComponent;
}

namespace sick
{

FrameComponent::FrameComponent(std::shared_ptr<const genistream::frame::IComponent> frameComponent)
  : m_component(frameComponent)
{
}

int FrameComponent::getWidth() const
{
  return m_component->getWidth();
}

int FrameComponent::getDeliveredHeight() const
{
  return m_component->getDeliveredHeight();
}

void* FrameComponent::getData() const
{
  return m_component->getData();
}

CameraFrame::CameraFrame(const std::shared_ptr<genistream::frame::IFrame> frame)
  : m_frame(frame)
{
}

std::string CameraFrame::getFrameID() const
{
  uint64_t frameId = m_frame->getFrameId();
  return std::to_string(frameId);
}

std::shared_ptr<IFrameComponent> CameraFrame::getRangeComponent() const
{
  return std::static_pointer_cast<IFrameComponent>(std::make_shared<FrameComponent>(m_frame->getRange()));
}

std::shared_ptr<IFrameComponent> CameraFrame::getIntensityComponent() const
{
  return std::static_pointer_cast<IFrameComponent>(std::make_shared<FrameComponent>(m_frame->getIntensity()));
}

std::shared_ptr<IFrameComponent> CameraFrame::getDepthComponent() const
{
  return std::static_pointer_cast<IFrameComponent>(std::make_shared<FrameComponent>(m_frame->getRange()));
}

std::shared_ptr<IFrameComponent> CameraFrame::getImuComponent() const
{
  // IMU data is a customized component
  // Therefore, the call for component query is different to range, intensity and depth.
  return std::static_pointer_cast<IFrameComponent>(
    std::make_shared<FrameComponent>(m_frame->getComponent(genistream::ComponentId::IMU_BASIC)));
}

uint64_t CameraFrame::getTimestamp() const
{
  auto buffer = m_frame->getBuffer().lock();
  if (buffer) {
    return buffer->getTimeStamp();
  }
  return 0;
}

} // namespace sick