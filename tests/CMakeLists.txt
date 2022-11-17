add_executable(robotino2test main.cpp)

set_target_properties(robotino2test PROPERTIES VERSION ${PROJECT_VERSION})

target_link_libraries(robotino2test PUBLIC robotino2wrapper)

install(TARGETS robotino2test
        EXPORT ${PROJECT_NAME}Targets
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}/test" COMPONENT bin)