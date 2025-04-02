// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_SRC_PLUGIN_CAMERAPLUGINBASE_HPP
#define SICK_PUBLISHER_SRC_PLUGIN_CAMERAPLUGINBASE_HPP

namespace sick
{
class CameraPluginBase
{
public:
  virtual ~CameraPluginBase() = default;
  virtual void initialize() = 0;
  virtual void execute() = 0;
};

} // namespace sick

#endif // SICK_PUBLISHER_CAMERAPLUGINBASE_HPP
