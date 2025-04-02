// Copyright (c) 2024 SICK AG, Waldkirch
// SPDX-License-Identifier: Unlicense

#ifndef SICK_PUBLISHER_SRC_BASE_CMDLINEFLAGS_HPP
#define SICK_PUBLISHER_SRC_BASE_CMDLINEFLAGS_HPP

#include <cstdint>
#include <string>

namespace sick
{
/// Macro for referencing flags
#define FLAG(name) FLAGS_##name

/// Macros for declaring flags
#define DECLARE_bool(name) extern bool FLAG(name)
#define DECLARE_int32(name) extern int32 FLAG(name)
#define DECLARE_float(name) extern float FLAG(name)
#define DECLARE_string(name) extern std::string FLAG(name)

/// Macros for defining flags
#define DEFINE_bool(name, def_val, doc) bool FLAG(name) = (def_val)
#define DEFINE_int32(name, def_val, doc) int32 FLAG(name) = (def_val)
#define DEFINE_float(name, def_val, doc) float FLAG(name) = (def_val)
#define DEFINE_string(name, def_val, doc) std::string FLAG(name) = (def_val)

/// @brief Parses |str| for a 32-bit signed integer and writes the result to |value|, if successful.
bool parseInt32(const std::string& src, const char* str, int32_t* value);

/// @brief Parses a string for a bool flag in the form of '--flag=value' or '--flag'.
/// In the former case, the value is taken as true, as long as it does not start_
/// with '0', 'f', or 'F'. In the latter case, the value is taken as true.
bool parseBoolFlag(const char* str, const char* flag, bool* value);

/// @brief Parses a string for an int32 flag, in the form of '--flag=value'.
/// On success it stores the value of the flag in |value|.
bool parseInt32Flag(const char* str, const char* flag, int32_t* value);

/// @brief Parses a string for a string flag, in the form of '--flag=value'.
/// On success it stores the value of the flag in |value|.
bool parseStringFlag(const char* str, const char* flag, std::string* value);

/// @brief Returns true, if the string matches the |flag|.
bool isFlag(const char* str, const char* flag);

} // namespace sick

#endif // SICK_PUBLISHER_SRC_BASE_CMDLINEFLAGS_HPP
