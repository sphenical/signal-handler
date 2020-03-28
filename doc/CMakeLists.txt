project (signal-handler-docs)
cmake_minimum_required (VERSION 3.13)

include (FindDoxygen)
if (NOT ${DOXYGEN_FOUND})
    message (FATAL_ERROR
        "Cannot find doxygen, this is needed for building the documentation.")
endif ()

configure_file (
    ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in
    ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)

set (DOXYGEN_INCLUDE_DIRS
    -isystem ${CMAKE_SOURCE_DIR}/src)

add_custom_target (doc ALL
    COMMAND ${DOXYGEN_EXECUTABLE}
        ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile ${DOXYGEN_INCLUDE_DIRS}
    COMMENT "Generating API documentation.")

if (NOT ${SIGNALHANDLER_DISABLE_INSTALLS})
    install (DIRECTORY
        ${CMAKE_CURRENT_BINARY_DIR}/html DESTINATION ${CMAKE_INSTALL_DOCDIR})

endif ()
