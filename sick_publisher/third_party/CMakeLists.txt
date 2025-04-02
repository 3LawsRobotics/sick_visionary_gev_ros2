# Copyright (c) 2024 SICK AG, Waldkirch
# SPDX-License-Identifier: Unlicense

# Third party libraries are built using ExternalProject_Add(), which per default
# doesn't inherit any variables from the parent project, Thus, we set all
# necessary variables and settings from the parent.
set(SICKPublisher_INSTALL_PREFIX ${EXTERNAL_INSTALL_PREFIX}/${CMAKE_BUILD_TYPE})
set(SICKPublisher_DEFAULT_ARGS
        -DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
        -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
        -DCMAKE_INSTALL_PREFIX:PATH=${SICKPublisher_INSTALL_PREFIX}
        -DCMAKE_C_COMPILER:STRING=${CMAKE_C_COMPILER}
        -DCMAKE_CXX_COMPILER:STRING=${CMAKE_CXX_COMPILER}
        -DCMAKE_GENERATOR:STRING=${CMAKE_GENERATOR}
        -DCMAKE_MAKE_PROGRAM:FILEPATH=${CMAKE_MAKE_PROGRAM}
        -DCMAKE_PREFIX_PATH:PATH=${SICKPublisher_INSTALL_PREFIX};${CMAKE_PREFIX_PATH}
        -DCMAKE_TOOLCHAIN_FILE:FILEPATH=${CMAKE_TOOLCHAIN_FILE})

# Setup project dependencies
include(ExternalProject)
include(Projects)

# Add TinyXML2 library
add_library(third_party_tinyxml2
  tinyxml2/8.0.0/tinyxml2.cpp
  tinyxml2/8.0.0/tinyxml2.h)
add_library(third_party::tinyxml2 ALIAS third_party_tinyxml2)
target_include_directories(third_party_tinyxml2
  SYSTEM PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/tinyxml2/8.0.0>
    $<INSTALL_INTERFACE:tinyxml2/8.0.0>)
install(TARGETS third_party_tinyxml2 EXPORT ${PROJECT_NAME}Targets)

option(USE_SYSTEM_GENISTREAM "Use installed version of GenIStream package" OFF)
if(USE_SYSTEM_GENISTREAM)
  find_package(GenIStream REQUIRED)
else()
  set(as_subproject "GenIStream;${as_subproject}" PARENT_SCOPE)
  include(External_genistream)
endif()

option(USE_SYSTEM_YAMLCPP "Use installed version of yaml-cpp package" OFF)
if(USE_SYSTEM_YAMLCPP)
  find_package(yaml-cpp REQUIRED)
else()
  set(as_subproject "yaml-cpp;${as_subproject}" PARENT_SCOPE)
  include(External_yamlcpp)
endif()
