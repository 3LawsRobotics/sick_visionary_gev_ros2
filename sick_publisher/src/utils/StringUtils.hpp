// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_SRC_UTILS_STRINGUTILS_HPP
#define SICK_PUBLISHER_SRC_UTILS_STRINGUTILS_HPP

#include <string>
#include <vector>

/// @file StringUtils.hpp
/// @brief String utility functions used throughout the codebase

namespace sick
{

enum WhitespaceHandling
{
  WHITESPACE_KEEP,
  WHITESPACE_TRIM
};

enum SplitResult
{
  SPLIT_WANT_ALL,
  SPLIT_WANT_NONEMPTY
};

enum TrimPositions
{
  TRIM_NONE = 0,
  TRIM_LEADING = 1 << 0,
  TRIM_TRAILING = 1 << 1,
  TRIM_ALL = TRIM_LEADING | TRIM_TRAILING
};

/// Whitespace for char tab, line tab, form feed (FF) and space
#define WHITESPACE_ASCII_NO_CR_LF 0x09, 0x0B, 0x0C, 0x20

/// Whitespace for line feed (LF) and carriage return (CR)
#define WHITESPACE_ASCII WHITESPACE_ASCII_NO_CR_LF, 0x0A, 0x0D

/// Quotation mark and apostrophe
#define QUOTATION_ASCII WHITESPACE_ASCII, 0x22, 0x27

constexpr char kWhitespaceASCII[] = {WHITESPACE_ASCII, 0};
constexpr char kQuotationASCII[] = {QUOTATION_ASCII, 0};

std::string trimString(const std::string& input, const std::string& trimChars, TrimPositions positions);

std::vector<std::string> splitString(const std::string& input,
                                     const std::string& separator,
                                     WhitespaceHandling whitespace = WHITESPACE_TRIM,
                                     SplitResult splitResult = SPLIT_WANT_NONEMPTY);

bool isStringAscii(const std::wstring& str);

bool isStringAscii(const std::string& str);

template <typename Char>
bool isAsciiLower(const Char c)
{
  return 'a' <= c && c <= 'z';
}

inline char toLowerAscii(const char c)
{
  return ('A' <= c && c <= 'Z') ? (c + ('a' - 'A')) : c;
}

std::string toLowerAscii(const std::string& str);

template <typename Char>
bool isAsciiUpper(Char c)
{
  return 'A' <= c && c <= 'Z';
}

inline char toUpperAscii(const char c)
{
  return ('a' <= c && c <= 'z') ? (c + ('A' - 'a')) : c;
}

std::string toUpperAscii(const std::string& str);
} // namespace sick

#endif // UTILS_STRINGUTILS_HPP_
