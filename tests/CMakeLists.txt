add_executable(Robotino2Test main.cpp)

set_target_properties(Robotino2Test PROPERTIES VERSION ${PROJECT_VERSION})

target_link_libraries(Robotino2Test PUBLIC Robotino2Lib)

install(TARGETS Robotino2Test
        EXPORT ${PROJECT_NAME}Targets
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}/test" COMPONENT bin)