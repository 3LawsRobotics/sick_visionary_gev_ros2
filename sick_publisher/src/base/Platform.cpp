// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include "base/Platform.hpp"

#include <string>

namespace sick
{
std::string getPlatformString()
{
  std::string os{};
  std::string arch{};
#if defined(SICK_OS_LINUX)
  os = "linux";
#elif defined(SICK_OS_WINDOWS)
  os = "windows";
#endif

#if defined(SICK_PLATFORM_ARM64_BIT)
  arch = "aarch64";
#elif defined(SICK_PLATFORM_X64_BIT)
  arch = "x64";
#endif

  if (os.empty() || arch.empty())
  {
    return {};
  }

  return os + "_" + arch;
}
} // namespace sick