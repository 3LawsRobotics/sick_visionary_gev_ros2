# Copyright (c) 2024 SICK AG, Waldkirch SPDX-License-Identifier: Unlicense

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

########## Camera adapter GeniStream ##########
add_library(GenIStreamAdapter
        ${PROJECT_SOURCE_DIR}/adapter/GenIStreamSDK/CameraDiscovery.cpp
        ${PROJECT_SOURCE_DIR}/adapter/GenIStreamSDK/CameraFrame.cpp
        ${PROJECT_SOURCE_DIR}/adapter/GenIStreamSDK/CameraControl.cpp)
add_library(sick::GenIStreamAdapter ALIAS GenIStreamAdapter)

set_target_properties(GenIStreamAdapter PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(GenIStreamAdapter
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/adapter>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)

target_link_libraries(GenIStreamAdapter GenIStream::GenIStream)

ament_target_dependencies(GenIStreamAdapter cv_bridge)
ament_target_dependencies(GenIStreamAdapter  ament_index_cpp)
install(TARGETS GenIStreamAdapter)

########## Experimental: GenIStream plugin ##########
add_library(GenIStreamPlugin SHARED plugin/GenIStreamPlugin.cpp)
target_include_directories(GenIStreamPlugin PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>)
ament_target_dependencies(GenIStreamPlugin pluginlib)
install(TARGETS GenIStreamPlugin
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${INSTALL_ARCHIVE_DIR}
  LIBRARY DESTINATION ${INSTALL_LIBRARY_DIR}
  RUNTIME DESTINATION ${INSTALL_RUNTIME_DIR})

########## SICKPublisher Lib ##########
add_library(SICKPublisher SHARED)
target_sources(
  SICKPublisher
  PUBLIC base/CmdLineFlags.cpp
         base/CmdLineFlags.hpp
         base/ErrorTrace.cpp
         base/Logging.cpp
         base/Logging.hpp
         base/Platform.cpp
         base/Platform.hpp
         base/Result.cpp
         base/Windows.hpp
         system/Environment.cpp
         system/Environment.hpp
         utils/FrameTransformations.cpp
         utils/StringUtils.cpp
         utils/StringUtils.hpp
         utils/TransformUtils.hpp
         utils/TransformUtils.cpp
         utils/ParamsFileUtils.hpp
         utils/ParamsFileUtils.cpp
         utils/PointCloudUtils.hpp
         utils/PointCloudUtils.cpp
         CameraInfo.cpp
         CameraParameters.cpp)
add_library(sick::SICKPublisher ALIAS SICKPublisher)

ament_target_dependencies(SICKPublisher
  rclcpp
  rclcpp_components
  std_msgs
  diagnostic_msgs
  sensor_msgs
  tf2
  tf2_ros
  cv_bridge)

target_include_directories(SICKPublisher
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
         $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

set_target_properties(SICKPublisher PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_compile_definitions(
  SICKPublisher PUBLIC -DSICK_ENABLE_LOGGING=$<BOOL:${SICK_ENABLE_LOGGING}>)

target_link_libraries(SICKPublisher
  sick::GenIStreamAdapter
  $<$<CXX_COMPILER_ID:MSVC>:wsock32>
  $<$<CXX_COMPILER_ID:MSVC>:ws2_32>
  $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>:stdc++fs>
  yaml-cpp::yaml-cpp)

install(TARGETS SICKPublisher
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${INSTALL_ARCHIVE_DIR}
  LIBRARY DESTINATION ${INSTALL_LIBRARY_DIR}
  RUNTIME DESTINATION ${INSTALL_RUNTIME_DIR})

# ######### SICKPublisher-Node ##########
target_sources(SICKPublisher_component PUBLIC SICKPublisher.cpp)
add_library(sick::SICKPublisher_component ALIAS SICKPublisher_component)

target_link_libraries(SICKPublisher_component SICKPublisher)

ament_target_dependencies(SICKPublisher_component
  cv_bridge rclcpp rclcpp_components sensor_msgs message_filters)

target_include_directories(SICKPublisher_component
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
         $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/adapter>
         $<INSTALL_INTERFACE:include>)

install(TARGETS SICKPublisher_component
  EXPORT SICKPublisher_componentTargets
  ARCHIVE DESTINATION ${INSTALL_ARCHIVE_DIR}
  LIBRARY DESTINATION ${INSTALL_LIBRARY_DIR}
  RUNTIME DESTINATION ${INSTALL_RUNTIME_DIR})

# ######### PointCloudPublisher Node ##########
target_sources(PointCloudPublisher PUBLIC PointCloudPublisher.cpp)
add_library(sick::PointCloudPublisher ALIAS PointCloudPublisher)

target_link_libraries(PointCloudPublisher SICKPublisher)

ament_target_dependencies(PointCloudPublisher
  cv_bridge rclcpp rclcpp_components sensor_msgs message_filters)

target_include_directories(PointCloudPublisher
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
         $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/adapter>
         $<INSTALL_INTERFACE:include>)

install(TARGETS PointCloudPublisher
  EXPORT PointCloudPublisherTargets
  ARCHIVE DESTINATION ${INSTALL_ARCHIVE_DIR}
  LIBRARY DESTINATION ${INSTALL_LIBRARY_DIR}
  RUNTIME DESTINATION ${INSTALL_RUNTIME_DIR})

# Manual Composition
add_executable(manual_composition ManualComposition.cpp)

target_sources(manual_composition PUBLIC PointCloudPublisher.cpp SICKPublisher.cpp)

target_link_libraries(manual_composition SICKPublisher yaml-cpp::yaml-cpp)

target_include_directories(manual_composition
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
         $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/adapter>
         $<INSTALL_INTERFACE:include>)

ament_target_dependencies(manual_composition rclcpp)

# Install transitive dependencies.
# This has to be cleaned up some time in the future using install(IMPORTED_RUNTIME_ARTIFACTS)
get_target_property(GENICAM_LIBS GenICam::GenICam INTERFACE_LINK_LIBRARIES)
foreach(lib ${GENICAM_LIBS})
  get_target_property(DLL_LIB ${lib} IMPORTED_LOCATION)
  if(DLL_LIB)
    install(FILES ${DLL_LIB} DESTINATION ${INSTALL_LIBRARY_DIR})
  endif()
  get_target_property(IMP_LIB ${lib} IMPORTED_IMPLIB)
  if(IMP_LIB)
    install(FILES ${IMP_LIB} DESTINATION ${INSTALL_LIBRARY_DIR})
  endif()
endforeach()

# ROS2 run <..> won't find the executable if it is not installed in lib/
install(TARGETS manual_composition DESTINATION ${INSTALL_LIBRARY_DIR}/${PROJECT_NAME})
