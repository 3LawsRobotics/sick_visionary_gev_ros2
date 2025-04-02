// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_INCLUDE_BASE_ERRORTRACE_HPP
#define SICK_PUBLISHER_INCLUDE_BASE_ERRORTRACE_HPP

/// @file ErrorTrace.hpp
/// @brief ErrorTrace class for detailed error reporting in a ROS application
///
/// This class is designed to provide detailed error reporting and backtrace information for ROS
/// applications. Since ROS publisher nodes are built and installed locally using the colcon build
/// tool, it can be challenging to trace the exact location and cause of errors because the ROS
/// application cannot always be started from within an IDE with an attached debugger.
///
/// Hence this class helps to capture and log detailed information, including the error code,
/// message and the precise location in the code where the error occured.

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <base/Compiler.hpp>

namespace sick
{
/// @brief Enum class representing error codes.
enum class ErrorCode : uint32_t;

/// @brief Class for capturing and reporting detailed error information.
class SICK_NO_DISCARD ErrorTrace
{
public:
  /// @brief Create an ErrorTrace instance.
  /// @param code The error code. Taken from @file ErrorCode.hpp
  /// @param message The error message.
  /// @param file The file where the error occurred.
  /// @param function The function where the error occurred.
  /// @param line The line number where the error occurred.
  /// @return A std::unique_ptr to the created ErrorTrace instance.
  static std::unique_ptr<ErrorTrace> create(ErrorCode code,
                                            const std::string& message,
                                            const char* file,
                                            const char* function,
                                            int line);

  ErrorTrace(ErrorCode code, std::string message);

  /// @brief Struct to hold backtrace information.
  struct BacktraceRecord
  {
    const char* file;     ///< The file where the backtrace record was added.
    const char* function; ///< The function where the backtrace record was added.
    int line;             ///< The line number where the backtrace record was added.
  };

  /// @brief Append backtrace information.
  /// @param file The file where the backtrace record is added.
  /// @param function The function where the backtrace record is added.
  /// @param line The line number where the backtrace record is added.
  void appendBacktrace(const char* file, const char* function, int line);
  SICK_NO_DISCARD ErrorCode getCode() const;
  SICK_NO_DISCARD const std::string& getMessage() const;

  /// @brief Get the backtrace information.
  /// @return A vector of BacktraceRecord containing the backtrace information.
  SICK_NO_DISCARD const std::vector<BacktraceRecord>& getBacktrace() const;

private:
  ErrorCode m_code;
  std::string m_message;
  std::vector<BacktraceRecord> m_backtrace;
};

} // namespace sick

#endif //INCLUDE_BASE_ERRORTRACE_HPP_
