project (signal-handler-sources)
cmake_minimum_required (VERSION 3.13)

if (BUILD_SHARED_LIBS)
    add_library (signalhandler SHARED)
    set_property (
        TARGET signalhandler
        PROPERTY SOVERSION ${CMAKE_PROJECT_VERSION_MAJOR})
else ()
    add_library (signalhandler STATIC)
endif ()

add_library (signalhandler::signalhandler ALIAS signalhandler)

target_sources (signalhandler
    PRIVATE
        sig/handler.cpp
        sig/handler.hpp
)

target_include_directories (
    signalhandler SYSTEM
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

set_target_properties (signalhandler
    PROPERTIES
        VERSION ${CMAKE_PROJECT_VERSION}
        CXX_STANDARD 11
)

target_link_libraries (signalhandler
    INTERFACE
        ${CMAKE_THREAD_LIBS_INIT}
)

##############
# Installation
##############
if (NOT SIGNALHANDLER_DISABLE_INSTALLS)

    install (TARGETS signalhandler
        EXPORT signalhandler-config
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )

    export(EXPORT signalhandler-config NAMESPACE signalhandler::)

    install(EXPORT signalhandler-config
        NAMESPACE signalhandler::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/signalhandler
    )

    install (
        FILES
            sig/handler.hpp
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/sig
    )

    write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/signalhandler-config-version.cmake
        VERSION ${CMAKE_PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
    )

    install(
        FILES
            ${CMAKE_CURRENT_BINARY_DIR}/signalhandler-config-version.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/signalhandler
    )

endif ()

