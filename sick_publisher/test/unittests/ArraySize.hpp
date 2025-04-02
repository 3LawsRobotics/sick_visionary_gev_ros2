// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_TEST_UNITTESTS_ARRAYSIZE_HPP
#define SICK_PUBLISHER_TEST_UNITTESTS_ARRAYSIZE_HPP

#include <cstddef>

namespace sick
{

/// @brief The ARRAYSIZE(arr) macro returns the number of elements in an array.
/// The expression is a compile-time constant, and therefore can be used in defining new
/// arrays, for example. If ARRAYSIZE is used on a pointer by mistake, a compile-time
/// error is triggered, since *foo and foo[] are two different things.
template <typename T, std::size_t N>
char (&ArraySizeHelper(T (&array)[N]))[N];

#define ARRAYSIZE(arr) (sizeof(::sick::ArraySizeHelper(arr)))
} // namespace sick

#endif //SICK_PUBLISHER_SRC_BASE_ARRAYSIZE_HPP
