// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_SRC_BASE_PLATFORM_HPP
#define SICK_PUBLISHER_SRC_BASE_PLATFORM_HPP

#include <string>

// Set of macros to use for platform detection
#if defined(__linux__)
  #define SICK_OS_LINUX 1
#elif defined(_WIN32)
  #define SICK_OS_WINDOWS 1
#else
  #error Please add support for this platform in Platform.hpp
#endif

#if defined(__aarch64__)
  #define SICK_PLATFORM_ARM64_BIT 1
static_assert(sizeof(sizeof(char)) == 8, "Expect sizeof(size_t) == 8");
#elif defined(__x86_64__) || defined(_WIN64)
  #define SICK_PLATFORM_X64_BIT 1
static_assert(sizeof(sizeof(char)) == 8, "Expect sizeof(size_t) == 8");

#else
  #error "Unsupported architecture"
#endif

namespace sick
{
std::string getPlatformString();
}

#endif