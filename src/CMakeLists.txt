add_library(Robotino2Lib robotino2.cpp robotino2.hpp)

set_target_properties(Robotino2Lib PROPERTIES
                      VERSION ${PROJECT_VERSION}
                      PUBLIC_HEADER robotino2.hpp)

target_link_libraries(Robotino2Lib PUBLIC D:/Libraries/OpenRobotinoAPI/1/lib/win32/rec_core_lt.lib
                                          D:/Libraries/OpenRobotinoAPI/1/lib/win32/rec_cruizcore.lib
                                          D:/Libraries/OpenRobotinoAPI/1/lib/win32/rec_dataexchange_lt.lib
                                          D:/Libraries/OpenRobotinoAPI/1/lib/win32/rec_ea09_messages.lib
                                          D:/Libraries/OpenRobotinoAPI/1/lib/win32/rec_iocontrol_remotestate.lib
                                          D:/Libraries/OpenRobotinoAPI/1/lib/win32/rec_iocontrol_robotstate.lib
                                          D:/Libraries/OpenRobotinoAPI/1/lib/win32/rec_iocontrol_sercom.lib
                                          D:/Libraries/OpenRobotinoAPI/1/lib/win32/rec_nstar.lib
                                          D:/Libraries/OpenRobotinoAPI/1/lib/win32/rec_robotinoarm.lib
                                          D:/Libraries/OpenRobotinoAPI/1/lib/win32/rec_robotino_com.lib
                                          D:/Libraries/OpenRobotinoAPI/1/lib/win32/rec_robotino_com_dotnet.lib
                                          D:/Libraries/OpenRobotinoAPI/1/lib/win32/rec_robotino_com_java.lib
                                          D:/Libraries/OpenRobotinoAPI/1/lib/win32/rec_robotino_imagesender.lib
                                          D:/Libraries/OpenRobotinoAPI/1/lib/win32/rec_serialport.lib
                                          D:/Libraries/OpenRobotinoAPI/1/lib/win32/rec_robotino_com_c.lib
                                          D:/Libraries/OpenRobotinoAPI/1/lib/win32/rec_robotino_server.lib)

target_include_directories(Robotino2Lib PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/Robotino2>
                           "D:/Libraries/OpenRobotinoAPI/1/include"
                           "D:/Libraries/OpenRobotinoAPI/share/include")

install(TARGETS Robotino2Lib
        EXPORT Robotino2Targets
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/Robotino2" COMPONENT dev)