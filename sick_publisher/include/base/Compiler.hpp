// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_INCLUDE_BASE_COMPILER_HPP
#define SICK_PUBLISHER_INCLUDE_BASE_COMPILER_HPP

// Compiler detection
#if defined(__GNUC__)
  #define SICK_COMPILER_GCC 1
#elif defined(__clang__)
  #define SICK_COMPILER_CLANG 1
#elif defined(_MSC_VER)
  #define SICK_COMPILER_MSVC 1
#else
  #error Please add support for your compiler in Compiler.hpp
#endif

// Add support for nodiscard if available
#if defined(SICK_COMPILER_GCC) || defined(SICK_COMPILER_CLANG)
  #if !defined(__has_cpp_attribute)
    #define __has_cpp_attribute(name) 0
  #endif

  // Use either warn_unused_result on clanb or a c++1x exytension for C++14.
  // Also avoid warn_unused_result with GCC as it is only a function attribute as opposed
  // to a type attribute
  #if __has_cpp_attribute(warn_unused_result) && defined(SICK_COMPILER_CLANG)
    #define SICK_NO_DISCARD __attribute__((warn_unused_result))
  #elif __cplusplus >= 201703L && __has_cpp_attribute(nodiscard)
    #define SICK_NO_DISCARD [[nodiscard]]
  #endif
#elif defined(SICK_COMPILER_MSVC)
  #if __cplusplus >= 201703L && _MSC_VER >= 1911
    #define SICK_NO_DISCARD [[nodiscard]]
  #endif
#endif
#if !defined(SICK_NO_DISCARD)
  #define SICK_NO_DISCARD
#endif

#endif // BASE_COMPILER_HPP_
