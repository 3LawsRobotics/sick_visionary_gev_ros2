// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include "base/CmdLineFlags.hpp"

#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>

#include "base/Logging.hpp"

namespace sick
{
/// Parses a string as a cmdline flag. The string should have the format
/// '--flag=value'. If |defOptional| is true, the "=value" part can be
/// neglected.
/// Returns the value of the flag or nullptr, if the parsing failed
const char* parseFlagValue(const char* str, const char* flag, bool defOptional)
{
  // |str| and |flag| must not be nullptr.
  if (str == nullptr || flag == nullptr)
  {
    return nullptr;
  }

  // The flag must start_ with '--'
  auto flagStr = std::string("--") + std::string(flag);

  auto flagLen = flagStr.length();
  if (std::strncmp(str, flagStr.c_str(), flagLen) != 0)
  {
    return nullptr;
  }

  // Skip the flag name
  const auto* const flagEnd = str + flagLen;

  // If |defOptional| is set, it's ok to omit the '=value' part
  if (defOptional && (flagEnd[0] == '\0'))
  {
    return flagEnd;
  }

  // If |defOptional| is true and there are more chars after the flag name,
  // or if |defOptional| is false, there must be a '=' or single whitespace after the flag name
  if (flagEnd[0] != '=' && flagEnd[0] != ' ')
  {
    return nullptr;
  }

  // Returns the string after '='
  return flagEnd + 1;
}

bool parseInt32(const std::string& src, const char* str, int32_t* value)
{
  // Parses |str| as decimal integer
  char* end = nullptr;
  const auto longVal = std::strtol(str, &end, 10);

  // Did we consume all characters in the string?
  if (*end != '\0')
  {
    // No, we encountered an invalid character
    errLog() << src << " is expected to be a 32-bit integer, but actually "
             << "has the value: " << str;
    return false;
  }

  // Check, if the parsed value is in the range of an int32
  auto result = static_cast<int32_t>(longVal);
  if (std::numeric_limits<int64_t>::min() == longVal || std::numeric_limits<int64_t>::max() == longVal ||
      // Since strtol() returns LONG_MIN or LONG_MAX if the input overflows,
      // we check, if the parsed value overflows as an int32
      result != longVal)
  {
    errLog() << src << " is expected to be a 32-bit integer, but actually "
             << "has the value: " << str << " which overflows";
    return false;
  }

  *value = result;
  return true;
}

bool parseBoolFlag(const char* str, const char* flag, bool* value)
{
  // Gets the value of the flag as a string
  auto valueStr = parseFlagValue(str, flag, true);

  // Abort, if the parsing failed
  if (valueStr == nullptr)
  {
    return false;
  }

  // Convert the string value to a boolean
  *value = !(*valueStr == '0' || *valueStr == 'f' || *valueStr == 'F');
  return true;
}

bool parseInt32Flag(const char* str, const char* flag, int32_t* value)
{
  // Gets the value of the flag as a string
  auto valueStr = parseFlagValue(str, flag, false);

  // Abort, if the parsing failed
  if (valueStr == nullptr)
  {
    return false;
  }

  // Set |value| to the value of the flag
  return parseInt32(std::string("The value of flag --") + flag, valueStr, value);
}

bool parseStringFlag(const char* str, const char* flag, std::string* value)
{
  // Get the value of the flag as a string
  auto valueStr = parseFlagValue(str, flag, false);

  // Abort, if the parsing failed
  if (valueStr == nullptr)
  {
    return false;
  }

  *value = valueStr;
  return true;
}

bool isFlag(const char* str, const char* flag)
{
  return (parseFlagValue(str, flag, true) != nullptr);
}

} // namespace sick