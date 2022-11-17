add_library(robotino2wrapper robotino2.cpp)

set_target_properties(robotino2wrapper PROPERTIES
                      VERSION ${PROJECT_VERSION}
                      PUBLIC_HEADER robotino2.hpp
                      DEBUG_POSTFIX d)

string(REPLACE "\\" "/" OPENROBOTINOAPI_DIR "$ENV{OPENROBOTINOAPI_DIR}/1/lib/win32")

target_link_libraries(robotino2wrapper PUBLIC ${OPENROBOTINOAPI_DIR}/rec_core_lt.lib
                                          ${OPENROBOTINOAPI_DIR}/rec_robotino_com.lib)

include(GNUInstallDirs)

target_include_directories(robotino2wrapper PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
                           $ENV{OPENROBOTINOAPI_DIR}/1/include
                           $ENV{OPENROBOTINOAPI_DIR}/share/include)

install(TARGETS robotino2wrapper
        EXPORT ${PROJECT_NAME}Targets
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}" COMPONENT dev)