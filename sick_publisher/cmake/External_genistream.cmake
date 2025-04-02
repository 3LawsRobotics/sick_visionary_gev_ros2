# Copyright (c) 2024 SICK AG, Waldkirch
# SPDX-License-Identifier: Unlicense

set(libgenistream_name ${CMAKE_STATIC_LIBRARY_PREFIX}GenIStream${CMAKE_STATIC_LIBRARY_SUFFIX})
set(system_arch ${CMAKE_HOST_SYSTEM_NAME}_${CMAKE_HOST_SYSTEM_PROCESSOR})

set(genistream_wdir ${CMAKE_BINARY_DIR}/third_party)
execute_process(COMMAND ${CMAKE_COMMAND} -E tar xzf ${genistream_path}/archive.zip
  WORKING_DIRECTORY ${genistream_wdir})

########## GigE dependency ##########
function(create_genicam_import_target name)
  cmake_parse_arguments(THIS "" "NAME" "DLL_PATH;LIB_PATH" ${ARGN})

  if(NOT THIS_DLL_PATH AND NOT THIS_LIB_PATH)
    message(FATAL_ERROR "At least one of DLL_PATH or LIB_PATH must be provided.")
  endif()

  add_library(GenICam::${name} SHARED IMPORTED GLOBAL)
  if (THIS_DLL_PATH)
    set_target_properties(GenICam::${name} PROPERTIES IMPORTED_LOCATION "${THIS_DLL_PATH}")
  endif()
  if (THIS_LIB_PATH)
    set_target_properties(GenICam::${name} PROPERTIES IMPORTED_IMPLIB "${THIS_LIB_PATH}")
  endif()
endfunction()

## Assemble GenICam Reference Implementation naming scheme
set(GENAPI_VERSION "v3_1")

## Add different naming cases for the different platforms
if(WIN32)
  set(LIB_NAMES CLAllSerial CLProtocol GCBase GenApi GenCP Log log4cpp XmlParser)
  set(DLL_NAMES MathParser NodeMapData)
  set(COMPILER "MD_VC120")
elseif(UNIX)
  set(SO_NAMES CLAllSerial CLProtocol FirmwareUpdate GCBase GenApi Log log4cpp XmlParser MathParser NodeMapData)
  if(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "x86_64")
    set(COMPILER "gcc42")
  elseif(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "aarch64|arm64")
    set(COMPILER "gcc48")
  else()
    message(FATAL_ERROR "${CMAKE_HOST_SYSTEM_PROCESSOR} is not supported")
  endif()
endif()

set(GenICam_LIBRARIES)
foreach(name IN LISTS LIB_NAMES)
  set(lib_path ${genistream_wdir}/${genistream_version}/lib/bin/${system_arch}/${name}_${COMPILER}_${GENAPI_VERSION}${CMAKE_LINK_LIBRARY_SUFFIX})
  set(dll_path ${genistream_wdir}/${genistream_version}/lib/bin/${system_arch}/${CMAKE_SHARED_LIBRARY_PREFIX}${name}_${COMPILER}_${GENAPI_VERSION}${CMAKE_SHARED_LIBRARY_SUFFIX})
  create_genicam_import_target(${name} LIB_PATH ${lib_path} DLL_PATH ${dll_path})
  list(APPEND GenICam_LIBRARIES GenICam::${name})
endforeach()
foreach(name IN LISTS SO_NAMES)
  set(dll_path ${genistream_wdir}/${genistream_version}/lib/bin/${system_arch}/${CMAKE_SHARED_LIBRARY_PREFIX}${name}_${COMPILER}_${GENAPI_VERSION}${CMAKE_SHARED_LIBRARY_SUFFIX})
  create_genicam_import_target(${name} DLL_PATH ${dll_path})
  list(APPEND GenICam_LIBRARIES GenICam::${name})
endforeach()
set(GenICam_INCLUDE_DIRS "${genistream_wdir}/${genistream_version}/lib/library/CPP/include")
file(MAKE_DIRECTORY ${GenICam_INCLUDE_DIRS})

add_library(GenICam::GenICam INTERFACE IMPORTED GLOBAL)
set_target_properties(GenICam::GenICam PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES ${GenICam_INCLUDE_DIRS}
  INTERFACE_LINK_LIBRARIES "${GenICam_LIBRARIES}")

########## GenIStream dependency ##########
set(GenIStream_INCLUDE_DIR ${genistream_wdir}/${genistream_version}/include/include)
set(GenIStream_PUBLIC_DIR ${genistream_wdir}/${genistream_version}/include/public)
file(MAKE_DIRECTORY ${GenIStream_INCLUDE_DIR})
file(MAKE_DIRECTORY ${GenIStream_PUBLIC_DIR})
set(GenIStream_LIBRARIES ${genistream_wdir}/${genistream_version}/lib/bin/${system_arch}/${libgenistream_name})

add_library(GenIStream::GenIStream STATIC IMPORTED GLOBAL)
set_target_properties(GenIStream::GenIStream PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${GenIStream_INCLUDE_DIR};${GenIStream_PUBLIC_DIR}"
  INTERFACE_LINK_LIBRARIES GenICam::GenICam
  INTERFACE_COMPILE_DEFINITIONS GENISTREAM_LINKAGE_STATIC
  IMPORTED_LOCATION ${GenIStream_LIBRARIES}
  IMPORTED_IMPLIB ${GenIStream_LIBRARIES})
