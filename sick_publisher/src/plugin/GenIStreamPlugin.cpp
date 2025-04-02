// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include <memory>

#include <pluginlib/class_list_macros.hpp>

#include "CameraPluginBase.hpp"

namespace sick
{
class GenIStreamPlugin : public CameraPluginBase
{
public:
  void initialize() override
  {
    // TODO(xsowolk/DDDSP-1803): Initialization code
  }

  void execute() override
  {
    // TODO(xsowolk/DDDSP-1803): Execution code
  }
};
} // namespace sick

PLUGINLIB_EXPORT_CLASS(sick::GenIStreamPlugin, sick::CameraPluginBase)