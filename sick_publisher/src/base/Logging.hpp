// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_SRC_BASE_LOGGING_HPP
#define SICK_PUBLISHER_SRC_BASE_LOGGING_HPP

/// @file Logging.hpp
/// @brief SICK ROS2 publisher targets should never use iostream or printf directly:
///    - C++ iostream adds static initializations that should be avoided
///    - printf() and iostream do not show up in ROS context or mess up logging
///    - Those functions cannot separate between different log severities
///
/// As a general rule of thumb, logging should be kept to a minimum or entirely avoided, as ROS
/// itself adds a lot of logging information context already. However, there are some valid use
/// cases where logging is necessary such as hardware related issues (camera connection/sync) etc.
///
/// Regular logging is done using the [debug|info|warn|err]Log() function as follows:
///
///     warnLog() << "This is a " << severe << " message; // No newline needed
///
/// This call will generate a |Logging| object that is not stored anywhere and immediately calls
/// its destructor, which outputs the stored ostringstream at the right place. It also encapsulates
/// the required node handling (in ROS) which makes logging otherwise somewhat cumbersome.
///
/// In addition to that, this file also contains a SICK_DEBUG for so called "printf debugging" that
/// works in ROS context as well and also outputs the file, line and function name. It is called
/// as follows:
///
///     SICK_DEBUG(""); // Copy/Paste this in code to get log of execution
///
///     SICK_DEBUG("This is an error message"); // More info

#include <sstream>
#include <string>

#undef ERROR

namespace sick
{

constexpr char kSICKLoggingTag[] = "visionary_publisher";

/// @brief Defines different severity levels that can be used for logging messages
enum LogSeverity
{
  ERROR = 0, ///< Indicates an error message.
  WARNING,   ///< Indicates a warning message.
  INFO,      ///< Indicates an informational message.
  DEBUG      ///< Indicates a debug message.
};

class Logging
{
public:
  Logging(LogSeverity severity, const std::string& tag = kSICKLoggingTag);
  ~Logging();

  Logging(Logging&&) noexcept = default;
  Logging& operator=(Logging&&) = default;

  // Unfortunately, making these private/deleting those constructors don't work on ARM as
  // the compiler is unable to do copy elision
  Logging(const Logging& other);
  Logging& operator=(const Logging& other);

  template <typename T>
  Logging& operator<<(T&& value)
  {
    m_stream << value;
    return *this;
  }

  LogSeverity m_severity;
  std::ostringstream m_stream;

private:
  std::string m_tag;
};

Logging infoLog(const std::string& tag = kSICKLoggingTag);
Logging warnLog(const std::string& tag = kSICKLoggingTag);

Logging debugLog(const std::string& tag = kSICKLoggingTag);
Logging errLog(const std::string& tag = kSICKLoggingTag);

Logging debugLog(const char* file, const char* fn, int line, const std::string& tag = kSICKLoggingTag);
Logging errLog(const char* file, const char* fn, int line, const std::string& tag = kSICKLoggingTag);

/// Helper macro that create sophisticated debug log entry
#if defined(SICK_ENABLE_LOGGING)
  #define SICK_DEBUG(X) ::sick::debugLog(__FILE__, __func__, __LINE__) << X
  #define SICK_ERROR(X) ::sick::errLog(__FILE__, __func__, __LINE__) << X
#else
  #define SICK_DEBUG(X)
  #define SICK_ERROR(X)
#endif

} // namespace sick

#endif //SRC_BASE_LOGGING_HPP_
