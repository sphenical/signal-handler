cmake_minimum_required (VERSION 3.13)
project (signal-handler
    VERSION 1.3.0
    DESCRIPTION "A simple signal handler library to hook os signals."
    HOMEPAGE_URL https://github.com/sphenical/signal-handler
    LANGUAGES CXX
)

set (CMAKE_CXX_STANDARD_REQUIRED ON)

find_package (Threads REQUIRED)

include (FindPackageHandleStandardArgs)
include (GNUInstallDirs)
include (CMakePackageConfigHelpers)

option (BUILD_SHARED_LIBS "Disable the build of the shared library." ON)
option (SIGNALHANDLER_BUILD_TESTS "Enable testing." OFF)
option (SIGNALHANDLER_DISABLE_INSTALLS "Disable all installation targets." OFF)
option (SIGNALHANDLER_BUILD_DOCUMENTATION "Build the signal handler API documentation" OFF)
option (SIGNALHANDLER_BUILD_EXAMPLES "Build the signal handler examples" OFF)

set (SIGNALHANDLER_VENDOR "Sphenic Systems")
set (SIGNALHANDLER_BUGREPORT "info@sphenic.ch")
set (SIGNALHANDLER_README "README.md")
set (SIGNALHANDLER_LICENSE "LICENSE")

if (NOT ${SIGNALHANDLER_DISABLE_INSTALLS})
    set (CPACK_PACKAGE_NAME ${CMAKE_PROJECT_NAME})
    set (CPACK_PACKAGE_VENDOR ${SIGNALHANDLER_VENDOR})
    set (CPACK_PACKAGE_DESCRIPTION_SUMMARY ${CMAKE_PROJECT_DESCRIPTION})
    set (CPACK_PACKAGE_VERSION ${CMAKE_PROJECT_VERSION})
    set (CPACK_PACKAGE_VERSION_MAJOR ${CMAKE_PROJECT_VERSION_MAJOR})
    set (CPACK_PACKAGE_VERSION_MINOR ${CMAKE_PROJECT_VERSION_MINOR})
    set (CPACK_PACKAGE_VERSION_PATCH ${CMAKE_PROJECT_VERSION_PATCH})

    set (CPACK_RESOURCE_FILE_README ${CMAKE_CURRENT_SOURCE_DIR}/${SIGNALHANDLER_README})
    set (CPACK_RESOURCE_FILE_LICENSE ${CMAKE_CURRENT_SOURCE_DIR}/${SIGNALHANDLER_LICENSE})

    set (CPACK_GENERATOR "STGZ;TGZ")

    set (CPACK_SOURCE_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}-${CMAKE_PROJECT_VERSION}-src")
    set (CPACK_SOURCE_GENERATOR "TGZ")

    install (FILES
        ${CPACK_RESOURCE_FILE_README}
        ${CPACK_RESOURCE_FILE_LICENSE}
        DESTINATION ${CMAKE_INSTALL_DOCDIR})

    set (CPACK_SOURCE_IGNORE_FILES
        ".*~$"
        ".git"
        ".gitignore"
        ".*swp$"
        ".*zip$"
        ".*gz$"
        "Makefile.*"
        "^CPack.*"
        "_CPack_Packages"
        "CPackConfig.cmake"
        "CPackSourceConfig.cmake"
        "CMakeFiles"
        "CMakeCache.txt"
        "build.*/"
        ".ycm*"
        "tags")

endif ()

add_subdirectory (src)

if (SIGNALHANDLER_BUILD_TESTS)
    add_subdirectory (tests)
endif ()

if (SIGNALHANDLER_BUILD_EXAMPLES)
    add_subdirectory (examples)
endif ()

if (SIGNALHANDLER_BUILD_DOCUMENTATION)
    add_subdirectory (doc)
endif ()

if (NOT SIGNALHANDLER_DISABLE_INSTALLS)
    include (CPack)
endif ()

