add_library(Robotino2Lib robotino2.cpp)

set_target_properties(Robotino2Lib PROPERTIES
                      VERSION ${PROJECT_VERSION}
                      PUBLIC_HEADER robotino2.hpp)

string(REPLACE "\\" "/" OPENROBOTINOAPI_DIR "$ENV{OPENROBOTINOAPI_DIR}/1/lib/win32")

target_link_libraries(Robotino2Lib PUBLIC ${OPENROBOTINOAPI_DIR}/rec_core_lt.lib
                                          ${OPENROBOTINOAPI_DIR}/rec_robotino_com.lib)

include(GNUInstallDirs)

target_include_directories(Robotino2Lib PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
                           $ENV{OPENROBOTINOAPI_DIR}/1/include
                           $ENV{OPENROBOTINOAPI_DIR}/share/include)

install(TARGETS Robotino2Lib
        EXPORT ${PROJECT_NAME}Targets
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}" COMPONENT dev)