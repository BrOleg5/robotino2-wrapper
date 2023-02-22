find_package(Doxygen 1.8.0)

if(DOXYGEN_FOUND)
  set(DOXYGEN_REPEAT_BRIEF NO)
  set(DOXYGEN_EXAMPLE_PATH ${PROJECT_SOURCE_DIR}/example)
  set(DOXYGEN_QUIET YES)
  set(DOXYGEN_CLASS_GRAPH NO)
  set(DOXYGEN_COLLABORATION_GRAPH NO)

  doxygen_add_docs(
    doxygen
    ${PROJECT_SOURCE_DIR}
    COMMENT "Generate documentation"
  )

  install(DIRECTORY ${CMAKE_BINARY_DIR}/doc/html
    CONFIGURATIONS Release
    DESTINATION "doc"
  )
else()
  message(WARNING "Doxygen need to be installed to generate the doxygen documentation.")
endif()