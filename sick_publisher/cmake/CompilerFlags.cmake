# Copyright (c) 2024 SICK AG, Waldkirch
# SPDX-License-Identifier: Unlicense

option(EMIT_PERF_DIAGNOSTICS "Enable performance diagnostics" OFF)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Required for colcon

# Common compiler warnings used throughout the codebase
set(CLANG_WARNINGS
  $<$<BOOL:${EMIT_PERF_DIAGNOSTICS}>:-g> # Emit debug symbols
  -Wall                # Enable all default warnings
  -Wpedantic           # Enforce strict ISO C++ standard
  -Wextra              # Enable warnings not covered by -Wall
  $<$<COMPILE_LANGUAGE:CXX>:-Wsign-compare>  # Warning when comparing integers of different sign
  $<$<COMPILE_LANGUAGE:CXX>:-Wuninitialized> # Warning when accessing uninitialized data
  $<$<COMPILE_LANGUAGE:CXX>:-Wunused>        # Warning when variable are not used in scope
)

set(GCC_WARNINGS
  ${CLANG_WARNINGS}
  $<$<COMPILE_LANGUAGE:CXX>:-Woverloaded-virtual> # Additional warning for dynamic dispatch
)

# Common linker flags used throughout the codebase
set(CLANG_OPTIONS
  $<$<BOOL:${EMIT_PERF_DIAGNOSTICS}>:-Rpass-analysis=loop-vectorize> # Report missed vectorization
  $<$<STREQUAL:${ARCHITECTURE},arm>:-Ofast>
)

set(GCC_OPTIONS
  ${CLANG_OPTIONS}
  $<$<BOOL:${EMIT_PERF_DIAGNOSTICS}>:-fopt-info-vec-all> # Report missed vectorization
)

# Link this interface to set common compiler flags
add_library(sick_compiler_flags INTERFACE)
add_library(sick::compiler_flags ALIAS sick_compiler_flags)

target_compile_options(sick_compiler_flags BEFORE INTERFACE
  $<$<CXX_COMPILER_ID:AppleClang>:${CLANG_WARNINGS}>
  $<$<CXX_COMPILER_ID:AppleClang>:${CLANG_OPTIONS}>
  $<$<CXX_COMPILER_ID:Clang>:${CLANG_WARNINGS}>
  $<$<CXX_COMPILER_ID:Clang>:${CLANG_OPTIONS}>
  $<$<CXX_COMPILER_ID:GNU>:${GCC_OPTIONS}>
  $<$<CXX_COMPILER_ID:GNU>:${GCC_WARNINGS}>)

target_link_options(sick_compiler_flags INTERFACE
  $<$<CXX_COMPILER_ID:AppleClang>:${CLANG_OPTIONS}>
  $<$<CXX_COMPILER_ID:Clang>:${CLANG_OPTIONS}>
  $<$<CXX_COMPILER_ID:GNU>:${GCC_OPTIONS}>)

# Link this library to all build targets added later
link_libraries(sick::compiler_flags)
install(TARGETS sick_compiler_flags EXPORT ${PROJECT_NAME}Targets)