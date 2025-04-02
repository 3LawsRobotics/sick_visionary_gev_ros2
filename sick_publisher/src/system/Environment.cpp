// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include "system/Environment.hpp"

#include <cstdlib>

#include "base/Platform.hpp"
#include "base/Windows.hpp" // IWYU pragma: keep
#include "utils/StringUtils.hpp"

namespace sick
{
namespace
{
class EnvironmentImpl : public Environment
{
public:
  bool getEnvVar(const std::string& varName, std::string* pResult) override
  {
    if (getEnvVarImpl(varName, pResult))
    {
      return true;
    }

    // It can happen that some environment variables are uppercase, while other are lowercase.
    // Thus, we are looking to find a variable name with the reverse case, e.g. SICK_GENTL_LOGGING
    // may be sick_gentl_logging
    char firstChar = varName[0];
    std::string altCaseVar;
    if (isAsciiLower(firstChar))
    {
      altCaseVar = toUpperAscii(varName);
    }
    else if (isAsciiUpper(firstChar))
    {
      altCaseVar = toLowerAscii(varName);
    }
    else
    {
      return false;
    }

    return getEnvVarImpl(altCaseVar, pResult);
  }

  bool setEnvVar(const std::string& varName, const std::string& newValue) override
  {
    return setEnvVarImpl(varName, newValue);
  }

  bool unsetEnvVar(const std::string& varName) override { return unsetEnvVarImpl(varName); }

private:
  static bool getEnvVarImpl(const std::string& varName, std::string* pResult)
  {
#if defined(SICK_OS_LINUX)
    const char* envValue = getenv(varName.data());
    if (envValue == nullptr)
    {
      return false;
    }
    if (pResult != nullptr)
    {
      *pResult = envValue;
    }
    return true;
#elif defined(SICK_OS_WINDOWS)
    std::unique_ptr<char[]> value(new char[32767]);
    DWORD valueLength = ::GetEnvironmentVariable(varName.c_str(), value.get(), 32767);
    if (valueLength == 0)
    {
      return false;
    }
    if (pResult)
    {
      *pResult = value.get();
    }
    return true;
#else
  #error not implemented
#endif
  }

  static bool setEnvVarImpl(const std::string& varName, const std::string& newValue)
  {
#if defined(SICK_OS_LINUX)
    // On success, zero is returned
    return setenv(varName.data(), newValue.c_str(), 1) == 0;
#elif defined(SICK_OS_WINDOWS)
    // On success, a nonzero value is returned
    return !!SetEnvironmentVariableA(varName.c_str(), newValue.c_str());
#endif
  }

  static bool unsetEnvVarImpl(const std::string& varName)
  {
#if defined(SICK_OS_LINUX)
    // On success, zero is returned
    return unsetenv(varName.data()) == 0;
#elif defined(SICK_OS_WINDOWS)
    // On success, a nonzero value is returned
    return !!SetEnvironmentVariableA(varName.c_str(), nullptr);
#endif
  }
};
} // namespace

Environment::~Environment() = default;

std::unique_ptr<Environment> Environment::create()
{
  return std::make_unique<EnvironmentImpl>();
}

bool Environment::hasEnvVar(const std::string& varName)
{
  return getEnvVar(varName, nullptr);
}
} // namespace sick