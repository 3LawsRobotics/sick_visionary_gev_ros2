// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include "base/ErrorTrace.hpp"

#include <utility>

namespace sick
{
std::unique_ptr<ErrorTrace> ErrorTrace::create(sick::ErrorCode code,
                                               const std::string& message,
                                               const char* file,
                                               const char* function,
                                               int line)
{
  auto error = std::make_unique<ErrorTrace>(code, message);
  error->appendBacktrace(file, function, line);
  return error;
}

ErrorTrace::ErrorTrace(sick::ErrorCode code, std::string message)
  : m_code(code)
  , m_message(std::move(message))
{
}

void ErrorTrace::appendBacktrace(const char* file, const char* function, int line)
{
  BacktraceRecord record;
  record.file = file;
  record.function = function;
  record.line = line;
  m_backtrace.push_back(record);
}

ErrorCode ErrorTrace::getCode() const
{
  return m_code;
}

const std::string& ErrorTrace::getMessage() const
{
  return m_message;
}

const std::vector<ErrorTrace::BacktraceRecord>& ErrorTrace::getBacktrace() const
{
  return m_backtrace;
}

} // namespace sick