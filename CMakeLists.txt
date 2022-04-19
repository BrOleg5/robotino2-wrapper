cmake_minimum_required(VERSION 3.0)

project(Robotino2 VERSION 1.0 DESCRIPTION "Library for Festo Robotino 2")

# set cmake config install dirs
if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/cmake/Robotino2)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
  "Installation directory for CMake files")

include(GNUInstallDirs)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

option(BUILD_TESTS "Build and install tests programs." OFF)

add_subdirectory(src)

if(BUILD_TESTS)
        add_subdirectory(tests)
endif()

# Add all targets to the build-tree export set
if(BUILD_TESTS)
  export(TARGETS Robotino2Lib Robotino2Test
         FILE "${PROJECT_BINARY_DIR}/cmake/Robotino2Targets.cmake")
else()
  export(TARGETS Robotino2Lib
         FILE "${PROJECT_BINARY_DIR}/cmake/Robotino2Targets.cmake")
endif()

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE Robotino2)

# Create the Robotino2Config.cmake and Robotino2ConfigVersion.cmake files
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(cmake/Robotino2Config.cmake.in
               "${PROJECT_BINARY_DIR}/cmake/Robotino2Config.cmake" @ONLY)

# ... for the install tree
set(CONF_INCLUDE_DIRS "\${ROBOTINO2_CMAKE_DIR}/${CMAKE_INSTALL_INCLUDEDIR}")
configure_file(cmake/Robotino2Config.cmake.in
               "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/cmake/Robotino2Config.cmake" @ONLY)

# ... for both
configure_file(cmake/Robotino2ConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/cmake/Robotino2ConfigVersion.cmake" @ONLY)

# Install the Robotino2Config.cmake and Robotino2ConfigVersion.cmake files
install(FILES
        "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/cmake/Robotino2Config.cmake"
        "${PROJECT_BINARY_DIR}/cmake/Robotino2ConfigVersion.cmake"
        DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

# Install the export set for use with the install-tree
install(EXPORT Robotino2Targets
        DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)