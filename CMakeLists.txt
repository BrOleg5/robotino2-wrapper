cmake_minimum_required(VERSION 3.9)

project(Robotino2Wrapper VERSION 1.1.0 DESCRIPTION "Library for Festo Robotino 2")

option(BUILD_EXAMPLE "Build and install example programs." OFF)

add_subdirectory(src)

if(BUILD_EXAMPLE)
  add_subdirectory(example)
endif()

find_package(Doxygen)

if(DOXYGEN_FOUND)
  set(DOXYGEN_REPEAT_BRIEF NO)
  set(DOXYGEN_EXAMPLE_PATH ${PROJECT_SOURCE_DIR}/example)
  set(DOXYGEN_QUIET YES)
  set(DOXYGEN_CLASS_GRAPH NO)
  set(DOXYGEN_COLLABORATION_GRAPH NO)

  doxygen_add_docs(
    doxygen
    ${PROJECT_SOURCE_DIR}
    COMMENT "Generate documentation"
  )
else()
  message(WARNING "Doxygen need to be installed to generate the doxygen documentation.")
endif()

export(EXPORT ${PROJECT_NAME}Targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/cmake/${PROJECT_NAME}Targets.cmake"
)

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION "cmake"
)

include(CMakePackageConfigHelpers)

configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION "cmake"
)

write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION "cmake"
)