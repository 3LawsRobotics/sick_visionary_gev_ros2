// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include <base/Result.hpp>

namespace sick
{
namespace details
{
intptr_t createPayload(const void* ptr, Payload type)
{
  auto payload = reinterpret_cast<intptr_t>(ptr);
  assert((payload & 3) == 0);
  return payload | type;
}

Payload getPayload(intptr_t payload)
{
  return static_cast<Payload>(payload & 3);
}

} // namespace details
} // namespace sick