project (signal-handler-tests)
cmake_minimum_required (VERSION 3.2)

enable_testing ()

########
## GTest
########
find_path (GTEST_INCLUDE_DIR NAMES gtest/gtest.h)
find_library (GTEST_LIBRARY NAMES gtest gtest_main)
find_package_handle_standard_args (gtest DEFAULT_MSG
    GTEST_LIBRARY GTEST_INCLUDE_DIR)
mark_as_advanced (GTEST_LIBRARY GTEST_INCLUDE_DIR)
set (GTEST_LIBRARIES ${GTEST_LIBRARY})
set (GTEST_INCLUDE_DIRS ${GTEST_INCLUDE_DIR})

if (NOT ${GTEST_FOUND})
    message (FATAL_ERROR "Cannot find the gtest library.")
endif ()

######################
## Include directories
######################
include_directories (${CMAKE_CURRENT_SOURCE_DIR})
include_directories (SYSTEM
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_BINARY_DIR}/src
    ${GTEST_INCLUDE_DIRS})

########
## Tests
########
set (TEST_SOURCES
    runner.cpp
    basic.cpp)

##########
## Targets
##########
add_executable (runtests EXCLUDE_FROM_ALL ${TEST_SOURCES})

target_link_libraries (runtests
    signalhandler
    ${GTEST_LIBRARIES})

set_target_properties (runtests
    PROPERTIES
        CXX_STANDARD 11)

#######
## Test
#######
add_test (NAME alltests
    COMMAND runtests --gtest_output=xml:${CMAKE_BINARY_DIR}/signals_testresults.xml
    WORKING_DIRECTORY $<TARGET_FILE_DIR:runtests>)

#############################################
## Command "check" instead of "test",
## that way its possible to add dependencies
## to the target.
#############################################
add_custom_target (check
    COMMAND ${CMAKE_CTEST_COMMAND} -C $<CONFIGURATION>
    DEPENDS runtests)


