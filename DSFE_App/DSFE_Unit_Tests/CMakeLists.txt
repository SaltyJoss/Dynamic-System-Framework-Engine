# CMakeLists.txt for DSFE_Unit_Tests executable
cmake_minimum_required(VERSION 3.10)
project(DSFE_Unit_Tests LANGUAGES C CXX)

enable_testing()

add_executable(DSFE_Unit_Tests
    src/main.cpp
    src/AdaptiveMethodTests.cpp
    src/ConvergenceOrderTests.cpp
    src/ExplicitMethodTests.cpp
    src/ImplicitMethodTests.cpp
)

target_link_libraries(DSFE_Unit_Tests PRIVATE
    DSFE_Core
    gtest_main
)

include(GoogleTest)

gtest_discover_tests(
    DSFE_Unit_Tests
    DISCOVERY_MODE PRE_TEST
    DISCOVERY_TIMEOUT 10
)

# if (CMAKE_BUILD_TYPE STREQUAL "Debug")
#     gtest_discover_tests(
#         DSFE_Unit_Tests
#         DISCOVERY_MODE PRE_TEST
#         DISCOVERY_TIMEOUT 10
#     )
# else()
#     gtest_discover_tests(DSFE_Unit_Tests)
# endif()
