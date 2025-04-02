# Copyright (c) 2025 SICK AG, Waldkirch
# SPDX-License-Identifier: Unlicense

ExternalProject_Add(yaml-cpp
  URL ${yamlcpp_path}
  URL_MD5 ${yamlcpp_md5}
  INSTALL_DIR ${SICKPublisher_INSTALL_PREFIX}
  UPDATE_COMMAND ""
  CMAKE_ARGS -Wno-dev ${SICKPublisher_DEFAULT_ARGS}
        -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF)

ExternalProject_Get_Property(yaml-cpp install_dir)
set(yamlcpp_name ${CMAKE_STATIC_LIBRARY_PREFIX}yaml-cpp${CMAKE_STATIC_LIBRARY_SUFFIX})
set(yamlcpp_ROOT ${install_dir} CACHE INTERNAL "")

file(MAKE_DIRECTORY ${yamlcpp_ROOT}/include)
set(YAMLCPP_INCLUDE_DIRS ${yamlcpp_ROOT}/include)
set(YAMLCPP_LIBRARIES ${yamlcpp_ROOT}/lib/${yamlcpp_name})

add_library(yaml-cpp::yaml-cpp STATIC IMPORTED GLOBAL)
add_dependencies(yaml-cpp::yaml-cpp yaml-cpp)
set_target_properties(yaml-cpp::yaml-cpp PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES ${YAMLCPP_INCLUDE_DIRS}
  IMPORTED_LOCATION ${YAMLCPP_LIBRARIES})