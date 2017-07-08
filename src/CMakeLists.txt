project (signal-handler-sources)
cmake_minimum_required (VERSION 3.2)

include_directories (SYSTEM
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR})

##########
## Sources
##########
set (SIGNALHANDLER_SOURCES
    signals/handler.cpp)

set (SIGNALHANDLER_INSTALL_HEADERS
    signals/handler.hpp)

#####################
# Include directories
#####################
include_directories (SYSTEM
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR})

#########
# Targets
#########
if (${SIGNALHANDLER_BUILD_SHARED_LIBS})
    add_library (signalhandler SHARED ${SIGNALHANDLER_SOURCES})
    set_property (TARGET signalhandler
        PROPERTY SOVERSION ${SIGNALHANDLER_VERSION_MAJOR})
else ()
    add_library (signalhandler STATIC ${SIGNALHANDLER_SOURCES})
endif ()

set_target_properties (signalhandler PROPERTIES
    VERSION ${SIGNALHANDLER_VERSION}
    CXX_STANDARD 11)

target_link_libraries (signalhandler
    ${CMAKE_THREAD_LIBS_INIT})

##############
# Installation
##############
if (NOT ${SIGNALHANDLER_DISABLE_INSTALLS})

    install (TARGETS signalhandler
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR})

    install (FILES ${SIGNALHANDLER_INSTALL_HEADERS}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/signals)

endif ()

