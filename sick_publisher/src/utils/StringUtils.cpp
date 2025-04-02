// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#include "utils/StringUtils.hpp"

#include <cstddef>
#include <cstdint>

namespace sick
{
namespace
{
// Assuming that a pointer is the size of a *machine word*, then uintptr_t is an integer type that
// is also of machine word size
using MachineWord = uintptr_t;
const uintptr_t kMachineWordAlignmentMask = sizeof(MachineWord) - 1;

inline bool isAlignedToMachineWord(const void* pointer)
{
  return (reinterpret_cast<MachineWord>(pointer) & kMachineWordAlignmentMask) == 0u;
}

template <typename T>
inline T* alignToMachineWord(T* pointer)
{
  return reinterpret_cast<T*>(reinterpret_cast<MachineWord>(pointer) & ~kMachineWordAlignmentMask);
}

template <std::size_t size, typename CharType>
struct NonASCIIMask;

template <>
struct NonASCIIMask<4, char>
{
  static inline uint32_t value() { return 0x80808080U; }
};

template <>
struct NonASCIIMask<8, char>
{
  static inline uint64_t value() { return 0x8080808080808080ULL; }
};

template <>
struct NonASCIIMask<4, wchar_t>
{
  static inline uint32_t value() { return 0xFFFFFF80U; }
};

template <>
struct NonASCIIMask<8, wchar_t>
{
  static inline uint64_t value() { return 0xFFFFFF80FFFFFF80ULL; }
};

template <typename Char>
inline bool isStringAsciiImpl(const Char* pchars, std::size_t length)
{
  MachineWord allCharBits = 0;
  const Char* end = pchars + length;

  // align the input
  while (!isAlignedToMachineWord(pchars) && pchars != end)
  {
    allCharBits |= *pchars;
    ++pchars;
  }

  // compare the values of CPU word size
  const Char* wordEnd = alignToMachineWord(end);
  const std::size_t loopIncrement = sizeof(MachineWord) / sizeof(Char);
  while (pchars != wordEnd)
  {
    allCharBits |= *pchars;
    pchars += loopIncrement;
  }

  // process the remaining bytes
  while (pchars != end)
  {
    allCharBits |= *pchars;
    ++pchars;
  }

  MachineWord nonAsciiBitMask = NonASCIIMask<sizeof(MachineWord), Char>::value();
  return !(allCharBits & nonAsciiBitMask);
}

template <typename StringType>
StringType toLowerAsciiImpl(StringType str)
{
  StringType ret;
  ret.reserve(str.size());
  for (std::size_t idx = 0; idx < str.size(); idx++)
  {
    ret.push_back(toLowerAscii(str[idx]));
  }
  return ret;
}

template <typename StringType>
StringType toUpperAsciiImpl(StringType str)
{
  StringType ret;
  ret.reserve(str.size());
  for (std::size_t idx = 0; idx < str.size(); idx++)
  {
    ret.push_back(toUpperAscii(str[idx]));
  }
  return ret;
}
} // namespace

std::string trimString(const std::string& input, const std::string& trimChars, TrimPositions positions)
{
  if (input.empty())
  {
    return "";
  }

  auto begin = (positions & TRIM_LEADING) ? input.find_first_not_of(trimChars) : 0;
  auto end = (positions & TRIM_TRAILING) ? input.find_last_not_of(trimChars) + 1 : input.size();
  return input.substr(begin, end - begin);
}

std::vector<std::string> splitString(const std::string& input,
                                     const std::string& separator,
                                     WhitespaceHandling whitespace,
                                     SplitResult splitResult)
{
  std::vector<std::string> result;
  if (input.empty())
  {
    return result;
  }

  std::size_t start = 0;
  while (start != std::string::npos)
  {
    auto end = input.find_first_of(separator, start);
    std::string piece;
    if (end == std::string::npos)
    {
      piece = input.substr(start);
      start = std::string::npos;
    }
    else
    {
      piece = input.substr(start, end - start);
      start = end + 1;
    }

    if (whitespace == WHITESPACE_TRIM)
    {
      piece = trimString(piece, kWhitespaceASCII, TRIM_ALL);
    }

    if (splitResult == SPLIT_WANT_ALL || !piece.empty())
    {
      result.emplace_back(piece);
    }
  }

  return result;
}

bool isStringAscii(const std::wstring& str)
{
  return isStringAsciiImpl(str.data(), str.length());
}

bool isStringAscii(const std::string& str)
{
  return isStringAsciiImpl(str.data(), str.length());
}

std::string toLowerAscii(const std::string& str)
{
  return toLowerAsciiImpl<std::string>(str);
}

std::string toUpperAscii(const std::string& str)
{
  return toUpperAsciiImpl<std::string>(str);
}
} // namespace sick