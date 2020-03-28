project (signal-handler-tests)
cmake_minimum_required (VERSION 3.13)

enable_testing ()

find_package (GTest REQUIRED CONFIG)
include (GoogleTest)

add_executable (runtests EXCLUDE_FROM_ALL)
target_sources (runtests
    PRIVATE
        runner.cpp
        basic.cpp
)

target_link_libraries (runtests
    PRIVATE
        signalhandler::signalhandler
        GTest::gtest
        GTest::gmock
        GTest::gmock_main
)

set_target_properties (runtests
    PROPERTIES
        CXX_STANDARD 11
)

file (MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/results)
gtest_discover_tests (runtests
    WORKING_DIRECTORY $<TARGET_FILE_DIR:runtests>
    XML_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/results
)

#############################################
## Command "check" instead of "test",
## that way its possible to add dependencies
## to the target.
#############################################
add_custom_target (check
    COMMAND ${CMAKE_CTEST_COMMAND} -C $<CONFIGURATION>
    DEPENDS runtests)


