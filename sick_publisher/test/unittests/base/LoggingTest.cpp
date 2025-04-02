// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include "base/Logging.hpp"

#include <cstddef>
#include <ostream>
#include <regex>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace sick;

class LogMatcher : public testing::MatcherInterface<const std::string&>
{
public:
  explicit LogMatcher(const std::string& expectedLevel, const std::string& expectedMessage)
    : m_level(expectedLevel)
    , m_message(expectedMessage)
  {
  }

  bool MatchAndExplain(const std::string& actualLog, testing::MatchResultListener*) const override
  {
    // matches ROS style logging, e.g.
    // [ERROR] [1718812241.402671170] [SICK ROS Publisher]: This is my error message
    std::regex logPattern(R"(\[(\w+)\] \[\d+\.\d+\] \[(.*)\]: (.*)\n)");
    std::smatch match;

    if (std::regex_match(actualLog, match, logPattern))
    {
      // We match the correct logging level, the logging tag for this application and the message
      // and are not interested in the timestamp since this depends on the time of execution of this
      // particular test
      return match[1] == m_level && match[2] == kSICKLoggingTag && match[3] == m_message;
    }

    return false;
  }

  void DescribeTo(::std::ostream* os) const override
  {
    *os << "\"[" << m_level << "]"
        << " [xxxxxxxxxx.xxxxxxxxx] "
        << "[" << kSICKLoggingTag << "]: " << m_message << "\\n\"";
  }

  void DescribeNegationTo(::std::ostream* os) const override
  {
    *os << "\"[" << m_level << "]"
        << " [xxxxxxxxxx.xxxxxxxxx] "
        << "[" << kSICKLoggingTag << "]: " << m_message << "\\n\"";
  }

private:
  std::string m_level;
  std::string m_message;
};

int countOccurrences(const std::string& str, const std::string& subStr)
{
  int count = 0;
  size_t pos = str.find(subStr, 0);
  while (pos != std::string::npos)
  {
    count++;
    pos = str.find(subStr, pos + subStr.length());
  }
  return count;
}

testing::Matcher<const std::string&> matchesLog(const std::string& expectedLevel, const std::string& expectedMessage)
{
  return testing::MakeMatcher(new LogMatcher(expectedLevel, expectedMessage));
}

TEST(LoggingTest, CheckErrorLogging)
{
  testing::internal::CaptureStderr();
  {
    auto log = Logging(ERROR);
    log << "This is my message";
    ASSERT_TRUE(log.m_severity == ERROR);
  }
  auto capturedLog = testing::internal::GetCapturedStderr();
  ASSERT_THAT(capturedLog, matchesLog("ERROR", "This is my message"));
}

TEST(LoggingTest, CheckDebugLogging)
{
  testing::internal::CaptureStderr();
  {
    auto log = Logging(DEBUG);
    log << "This is another message";
    ASSERT_TRUE(log.m_severity == DEBUG);
  }
  auto capturedLog = testing::internal::GetCapturedStderr();
  ASSERT_THAT(capturedLog, matchesLog("DEBUG", "This is another message"));
}

TEST(LoggingTest, CheckInfoLogging)
{
  testing::internal::CaptureStderr();
  {
    auto log = Logging(INFO);
    log << "This is a third message";
    ASSERT_TRUE(log.m_severity == INFO);
  }
  auto capturedLog = testing::internal::GetCapturedStderr();
  ASSERT_THAT(capturedLog, matchesLog("INFO", "This is a third message"));
}

TEST(LoggingTest, CheckCopyConstruction)
{
  testing::internal::CaptureStderr();
  {
    auto log = Logging(DEBUG);
    log << "This should get copied";

    auto log2 = log;
    ASSERT_TRUE(log2.m_severity == DEBUG);
  }
  const auto logOutput = testing::internal::GetCapturedStderr();
  ASSERT_EQ(countOccurrences(logOutput, "DEBUG"), 2);
  ASSERT_EQ(countOccurrences(logOutput, "This should get copied"), 2);
}