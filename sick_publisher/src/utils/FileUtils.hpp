// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_SRC_UTILS_FILEUTILS_HPP
#define SICK_PUBLISHER_SRC_UTILS_FILEUTILS_HPP

#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace sick
{

/// @brief Read either entire file or nore than \p num_bytes into container.
/// The container is assumed to be contiguous with an element size equal to 1 byte. and offers at most
/// size(), reserve() as well as random access (e.g. std::vector<uint8_t>, std::string etc.)
template <typename Container>
bool readFile(std::filesystem::path filename,
              Container& out,
              std::size_t numBytes = std::numeric_limits<std::size_t>::max())
{
  static_assert(sizeof(out[0] == 1, "readFile() only accepts containers with byte-sized elements"));

  std::ifstream file(filename, std::ios::binary | std::ios::ate);
  if (!file.is_open())
  {
    return false;
  }

  auto fileSize = file.tellg();
  if (fileSize == -1)
  {
    return false;
  }

  auto readSize = std::min<std::size_t>(fileSize, numBytes);
  out.reserve(readSize);
  out.resize(readSize);

  file.seekg(0, std::ios::beg);
  file.read(reinterpret_cast<char*>(out.data()), readSize);
  file.close();

  return true;
}
} // namespace sick
#endif //SICK_PUBLISHER_SRC_UTILS_FILEUTILS_HPP
