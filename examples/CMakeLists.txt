project (signal-handler-examples)
cmake_minimum_required (VERSION 3.13)

add_executable (sighandle-example)
target_sources (sighandle-example
    PRIVATE
        sighandle.cpp
)

target_link_libraries (sighandle-example
    PRIVATE
        signalhandler::signalhandler
)

set_target_properties (
    sighandle-example
    PROPERTIES
        CXX_STANDARD 11)

if (NOT SIGNALHANDLER_DISABLE_INSTALLS)
    install (TARGETS
        sighandle-example
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif ()

