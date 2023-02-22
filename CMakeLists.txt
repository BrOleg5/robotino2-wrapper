cmake_minimum_required(VERSION 3.9)

project(Robotino2Wrapper VERSION 1.1.0 DESCRIPTION "Library for Festo Robotino 2")

option(BUILD_EXAMPLE "Build and install example programs." OFF)
option(BUILD_DOC "Build and install documentation." OFF)

include(GNUInstallDirs)

add_subdirectory(src)

if(BUILD_EXAMPLE)
  add_subdirectory(example)
endif()

if(BUILD_DOC)
  add_subdirectory(doc)
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