add_library(Robotino2Lib robotino2.cpp robotino2.hpp)

set_target_properties(Robotino2Lib PROPERTIES
                      VERSION ${PROJECT_VERSION}
                      PUBLIC_HEADER robotino2.hpp)

target_link_libraries(Robotino2Lib PUBLIC $ENV{OPENROBOTINOAPI_DIR}/1/lib/win32/rec_core_lt.lib
                                          $ENV{OPENROBOTINOAPI_DIR}/1/lib/win32/rec_cruizcore.lib
                                          $ENV{OPENROBOTINOAPI_DIR}/1/lib/win32/rec_dataexchange_lt.lib
                                          $ENV{OPENROBOTINOAPI_DIR}/1/lib/win32/rec_ea09_messages.lib
                                          $ENV{OPENROBOTINOAPI_DIR}/1/lib/win32/rec_iocontrol_remotestate.lib
                                          $ENV{OPENROBOTINOAPI_DIR}/1/lib/win32/rec_iocontrol_robotstate.lib
                                          $ENV{OPENROBOTINOAPI_DIR}/1/lib/win32/rec_iocontrol_sercom.lib
                                          $ENV{OPENROBOTINOAPI_DIR}/1/lib/win32/rec_nstar.lib
                                          $ENV{OPENROBOTINOAPI_DIR}/1/lib/win32/rec_robotinoarm.lib
                                          $ENV{OPENROBOTINOAPI_DIR}/1/lib/win32/rec_robotino_com.lib
                                          $ENV{OPENROBOTINOAPI_DIR}/1/lib/win32/rec_robotino_com_dotnet.lib
                                          $ENV{OPENROBOTINOAPI_DIR}/1/lib/win32/rec_robotino_com_java.lib
                                          $ENV{OPENROBOTINOAPI_DIR}/1/lib/win32/rec_robotino_imagesender.lib
                                          $ENV{OPENROBOTINOAPI_DIR}/1/lib/win32/rec_serialport.lib
                                          $ENV{OPENROBOTINOAPI_DIR}/1/lib/win32/rec_robotino_com_c.lib
                                          $ENV{OPENROBOTINOAPI_DIR}/1/lib/win32/rec_robotino_server.lib)

target_include_directories(Robotino2Lib PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
                           $ENV{OPENROBOTINOAPI_DIR}/1/include
                           $ENV{OPENROBOTINOAPI_DIR}/share/include)

install(TARGETS Robotino2Lib
        EXPORT Robotino2Targets
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}" COMPONENT dev)