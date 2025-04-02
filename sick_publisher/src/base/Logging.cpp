// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include "base/Logging.hpp"

#include <string>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "base/Platform.hpp" // IWYU pragma: keep

#if defined(SICK_OS_WINDOWS)
  #include <cstdio>
#endif

namespace sick
{
namespace
{
rclcpp::Logger::Level severityTag(LogSeverity tag)
{
  using LogLevel = rclcpp::Logger::Level;
  switch (tag)
  {
    case LogSeverity::DEBUG:
      return LogLevel::Debug;
    case LogSeverity::INFO:
      return LogLevel::Info;
    case LogSeverity::WARNING:
      return LogLevel::Warn;
    case LogSeverity::ERROR:
      return LogLevel::Error;
    default:
      return LogLevel::Unset;
  }
}

#if defined(SICK_OS_WINDOWS)
const char* toString(LogSeverity tag)
{
  switch (tag)
  {
    case LogSeverity::DEBUG:
      return "DEBUG";
    case LogSeverity::INFO:
      return "INFO";
    case LogSeverity::WARNING:
      return "WARNING";
    case LogSeverity::ERROR:
      return "ERROR";
    default:
      return "";
  }
}
#endif

void rosLogMessage(LogSeverity tag, const std::string& message, const std::string& logTag)
{
  auto logger = rclcpp::get_logger(logTag);
  logger.set_level(severityTag(tag));
  switch (tag)
  {
    case LogSeverity::DEBUG:
      RCLCPP_DEBUG(logger, "%s", message.c_str());
      break;
    case LogSeverity::WARNING:
      RCLCPP_WARN(logger, "%s", message.c_str());
      break;
    case LogSeverity::ERROR:
      RCLCPP_ERROR(logger, "%s", message.c_str());
      break;
    case LogSeverity::INFO:
    default:
      RCLCPP_INFO(logger, "%s", message.c_str());
      break;
  }
}
} // namespace

Logging::Logging(LogSeverity severity, const std::string& tag)
  : m_severity(severity)
  , m_tag(tag)
{
}

Logging::~Logging()
{
  const auto message = m_stream.str();
  if (message.empty())
  {
    return;
  }

#if defined(SICK_OS_WINDOWS)
  // FIXME(xsowolk/DDDSP-1852): rclcpp::get_logger() is currently broken on Windows.
  // It simply fails hard (exit code 3) when it is called in Windows ROS2 iron environment. Following is a
  // workaround that mimics ROS logging facilities.
  std::FILE* outStream = stderr;
  std::fprintf(outStream, "%[%s] [000.000] [%s]: %s\n", toString(m_severity), kSICKLoggingTag, message.c_str());
  std::fflush(outStream);
#else
  rosLogMessage(m_severity, message, m_tag);
#endif
}

Logging::Logging(const Logging& other)
  : m_severity(other.m_severity)
{
  m_stream << other.m_stream.str();
}

Logging& Logging::operator=(const Logging& other)
{
  if (&other == this)
  {
    return *this;
  }

  m_severity = other.m_severity;
  m_stream << other.m_stream.rdbuf();

  return *this;
}

Logging debugLog(const std::string& tag)
{
  return {LogSeverity::DEBUG, tag};
}

Logging infoLog(const std::string& tag)
{
  return {LogSeverity::INFO, tag};
}

Logging warnLog(const std::string& tag)
{
  return {LogSeverity::WARNING, tag};
}

Logging errLog(const std::string& tag)
{
  return {LogSeverity::ERROR, tag};
}

Logging debugLog(const char* file, const char* fn, int line, const std::string& tag)
{
  auto message = debugLog(tag);
  message << file << ":" << line << "(" << fn << ")";
  return message;
}

Logging errLog(const char* file, const char* fn, int line, const std::string& tag)
{
  auto message = errLog(tag);
  message << file << ":" << line << "(" << fn << ")";
  return message;
}

} // namespace sick