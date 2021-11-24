add_executable(Robotino2Test main.cpp)

target_link_libraries(Robotino2Test PUBLIC Robotino2Lib)

# target_include_directories(Robotino2Test
#         PUBLIC "${PROJECT_SOURCE_DIR}/src")

if(INSTALL_TESTS)
        install(TARGETS Robotino2Test
                EXPORT Robotino2Targets
                RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}/test" COMPONENT bin)
endif()