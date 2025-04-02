# Copyright (c) 2024 SICK AG, Waldkirch
# SPDX-License-Identifier: Unlicense

find_package(Doxygen REQUIRED)

set(DOXYGEN_PROJECT_NAME ${APPLICATION_NAME})
set(DOXYGEN_PROJECT_BRIEF "SICK Visionary GEV ROS2")
set(DOXYGEN_OUTPUT_DIRECTORY ${sick_visionary_gev_ros2_BINARY_DIR}/doc)
set(DOXYGEN_RECURSIVE YES)
set(DOXYGEN_HAVE_DOT YES)
set(DOXYGEN_EXCLUDE */test/*)
set(DOXYGEN_EXTRACT_STATIC YES)
set(DOXYGEN_GENERATE_HTML YES)
set(DOXYGEN_HTML_COLORSTYLE LIGHT)
set(DOXYGEN_MARKDOWN_SUPPORT YES)
set(DOXYGEN_WARN_AS_ERROR NO)
set(DOXYGEN_PLANTUML_JAR_PATH /usr/share/plantuml/plantuml.jar)
set(DOXYGEN_SHOW_NAMESPACES YES)

cmake_path(GET sick_visionary_gev_ros2_SOURCE_DIR PARENT_PATH README_DIR)
set(DOXYGEN_USE_MDFILE_AS_MAINPAGE ${README_DIR}/README.md)

doxygen_add_docs(doc
  ${README_DIR}/README.md
  ${sick_visionary_gev_ros2_SOURCE_DIR}/adapter
  ${sick_visionary_gev_ros2_SOURCE_DIR}/include
  ${sick_visionary_gev_ros2_SOURCE_DIR}/src ALL
  COMMENT "Generate ROS2 Wrapper documentation")
install(DIRECTORY ${DOXYGEN_OUTPUT_DIRECTORY}/html
  DESTINATION ${INSTALL_DOC_DIR}/sick_visionary_gev_ros2)