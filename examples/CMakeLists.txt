project (signal-handler-examples)
cmake_minimum_required (VERSION 3.2)

include_directories (SYSTEM
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_BINARY_DIR}/src)

include_directories (${CMAKE_CURRENT_SOURCE_DIR})

set (SIGNALHANDLER_EXAMPLES)

add_executable (sighandle-example
    sighandle.cpp)

target_link_libraries (sighandle-example
    signalhandler)

list (APPEND SIGNALHANDLER_EXAMPLES sighandle-example)

set_target_properties (
    ${SIGNALHANDLER_EXAMPLES}
    PROPERTIES
        CXX_STANDARD 11)

if (NOT ${SIGNALHANDLER_DISABLE_INSTALLS})
    install (TARGETS
        ${SIGNALHANDLER_EXAMPLES}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif ()

