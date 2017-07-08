project (signal-handler)
cmake_minimum_required (VERSION 3.2)

set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_INCLUDE_CURRENT_DIR ON)

find_package (Threads REQUIRED)

include (FindPackageHandleStandardArgs)
include (GNUInstallDirs)

option (SIGNALHANDLER_BUILD_SHARED_LIBS "Disable the build of the shared library." ON)
option (SIGNALHANDLER_BUILD_TESTS "Enable testing." OFF)
option (SIGNALHANDLER_DISABLE_INSTALLS "Disable all installation targets." OFF)
option (SIGNALHANDLER_BUILD_DOCUMENTATION "Build the signal handler API documentation" OFF)
option (SIGNALHANDLER_BUILD_EXAMPLES "Build the signal handler examples" OFF)

set (SIGNALHANDLER_NAME "signal-handler")
set (SIGNALHANDLER_VENDOR "Sphenic Systems")
set (SIGNALHANDLER_URL "https://github.com/sphenical/signal-handler")
set (SIGNALHANDLER_BUGREPORT "info@sphenic.ch")
set (SIGNALHANDLER_DESCRIPTION_SUMMARY "A c++ POSIX signal handler wrapper library")
set (SIGNALHANDLER_README "README.md")
set (SIGNALHANDLER_LICENSE "LICENSE")

set (SIGNALHANDLER_VERSION_MAJOR "1")
set (SIGNALHANDLER_VERSION_MINOR "2")
set (SIGNALHANDLER_VERSION_PATCH "0")
set (SIGNALHANDLER_VERSION
    "${SIGNALHANDLER_VERSION_MAJOR}.${SIGNALHANDLER_VERSION_MINOR}.${SIGNALHANDLER_VERSION_PATCH}")

if (NOT ${SIGNALHANDLER_DISABLE_INSTALLS})
    set (CPACK_PACKAGE_NAME ${SIGNALHANDLER_NAME})
    set (CPACK_PACKAGE_VERSION ${SIGNALHANDLER_VERSION})
    set (CPACK_PACKAGE_VENDOR ${SIGNALHANDLER_VENDOR})
    set (CPACK_PACKAGE_DESCRIPTION_SUMMARY ${SIGNALHANDLER_DESCRIPTION_SUMMARY})
    set (CPACK_PACKAGE_VERSION_MAJOR ${SIGNALHANDLER_VERSION_MAJOR})
    set (CPACK_PACKAGE_VERSION_MINOR ${SIGNALHANDLER_VERSION_MINOR})
    set (CPACK_PACKAGE_VERSION_PATCH ${SIGNALHANDLER_VERSION_PATCH})

    set (CPACK_RESOURCE_FILE_README ${CMAKE_CURRENT_SOURCE_DIR}/${SIGNALHANDLER_README})
    set (CPACK_RESOURCE_FILE_LICENSE ${CMAKE_CURRENT_SOURCE_DIR}/${SIGNALHANDLER_LICENSE})
    set (CPACK_PACKAGE_DESCRIPTION_FILE ${CMAKE_CURRENT_SOURCE_DIR}/${SIGNALHANDLER_README})

    set (CPACK_GENERATOR "STGZ;TGZ")

    set (CPACK_SOURCE_PACKAGE_FILE_NAME "${SIGNALHANDLER_NAME}-${SIGNALHANDLER_VERSION}-src")
    set (CPACK_SOURCE_GENERATOR "TGZ")

    set (SIGNALHANDLER_INSTALL_DATADIR
        ${CMAKE_INSTALL_DATAROOTDIR}/${CPACK_PACKAGE_NAME})

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

if (${SIGNALHANDLER_BUILD_TESTS})
    add_subdirectory (tests)
endif ()

if (${SIGNALHANDLER_BUILD_EXAMPLES})
    add_subdirectory (examples)
endif ()

if (${SIGNALHANDLER_BUILD_DOCUMENTATION})
    add_subdirectory (doc)
endif ()

if (NOT ${SIGNALHANDLER_DISABLE_INSTALLS})
    include (CPack)
endif ()

