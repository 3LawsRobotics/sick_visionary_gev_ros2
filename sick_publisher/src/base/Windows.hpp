// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_SRC_BASE_WINDOWS_HPP
#define SICK_PUBLISHER_SRC_BASE_WINDOWS_HPP

#include "base/Platform.hpp"

/// @file Windows.hpp
/// @brief This header is intended to be a drop-in replacement for <Windows.h>, <WinSock2.h> and
/// related headers as it undefines a lot of names that are used within sick_publisher

#if defined(SICK_OS_WINDOWS)
  #include <cstdio>

  #ifndef __STDC__
    #define __STDC__ 1
    #pragma push_macro("_CRT_DECLARE_NONSTDC_NAMES")
    #ifdef _CRT_DECLARE_NONSTDC_NAMES
      #undef _CRT_DECLARE_NONSTDC_NAMES
    #endif
    #pragma push_macro("_CRT_INTERNAL_NONSTDC_NAMES")
    #undef _CRT_INTERNAL_NONSTDC_NAMES
    #include <direct.h>
    #include <io.h>
    #undef __STDC__
    #pragma pop_macro("_CRT_INTERNAL_NONSTDC_NAMES")
    #pragma pop_macro("_CRT_DECLARE_NONSTDC_NAMES")
  #else
    #include <direct.h>
    #include <io.h>
  #endif

  #if defined(min) || defined(max)
    #error The Windows.hpp header needs to be included by this header, otherwise \
NOMINMAX needs to be defined before including it.
  #endif

  #ifndef NOMINMAX
    #define NOMINMAX 1
  #endif

  #include <WinSock2.h>
  #include <Windows.h>

  #ifdef ERROR
    #undef ERROR
  #endif

  #ifdef NO_ERROR
    #undef NO_ERROR
  #endif

  #ifdef STRICT
    #undef STRICT
  #endif

#endif

#endif // BASE_WINDOWS_HPP_
