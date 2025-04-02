// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_SRC_SYSTEM_ENVIRONMENT_HPP
#define SICK_PUBLISHER_SRC_SYSTEM_ENVIRONMENT_HPP

#include <memory>
#include <string>

namespace sick
{
class Environment // NOLINT: Allow missing case style for this abstract class
{
public:
  virtual ~Environment();

  static std::unique_ptr<Environment> create();

  /// @brief Gets an environment variable and returns it in |result|
  /// @return Returns false, if the variable is not set
  virtual bool getEnvVar(const std::string& varName, std::string* pResult) = 0;

  /// @brief Returns true, if the environment variable is set
  virtual bool hasEnvVar(const std::string& varName);

  /// @brief Set an environment variable
  /// @return Returns true on success, otherwise returns false
  virtual bool setEnvVar(const std::string& varName, const std::string& newValue) = 0;

  /// @brief Returns true on success, otherwise returns false
  virtual bool unsetEnvVar(const std::string& varName) = 0;
};

} // namespace sick

#endif // SYSTEM_ENVIRONMENT_HPP_
