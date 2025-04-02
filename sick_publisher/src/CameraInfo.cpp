// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include "sick_publisher/CameraInfo.hpp"

namespace sick
{
CameraInfo::CameraInfo(const std::string& id,
                       const std::string& model,
                       const std::string& ipAddress,
                       const std::string& serialNumber)
  : m_id(id)
  , m_model(model)
  , m_ipAddress(ipAddress)
  , m_serialNumber(serialNumber)
  , m_isConnected(false)
{
}
} // namespace sick