add_executable(move_forward move_forward.cpp)

set_target_properties(move_forward PROPERTIES VERSION ${PROJECT_VERSION})

target_link_libraries(move_forward PUBLIC robotino2wrapper)

install(TARGETS move_forward
  EXPORT ${PROJECT_NAME}Targets
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}/example" COMPONENT bin
)