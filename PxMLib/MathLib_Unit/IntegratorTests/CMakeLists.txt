# CMakeLists.txt for MathLib_Unit_Tests executable
cmake_minimum_required(VERSION 3.10)
project(MathLib_IntegratorTests LANGUAGES C CXX)

enable_testing()

add_executable(MathLib_IntegratorTests
    main.cpp
    AdaptiveMethodTests.cpp
    ConvergenceOrderTests.cpp
    ExplicitMethodTests.cpp
    ImplicitMethodTests.cpp
)

target_link_libraries(MathLib_IntegratorTests PRIVATE
    MathLib
    gtest_main
)

target_include_directories(MathLib_IntegratorTests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Common
)

include(GoogleTest)

gtest_discover_tests(
    MathLib_IntegratorTests
    DISCOVERY_MODE PRE_TEST
    DISCOVERY_TIMEOUT 10
)

# if (CMAKE_BUILD_TYPE STREQUAL "Debug")
#     gtest_discover_tests(
#         MathLib_Unit_Tests
#         DISCOVERY_MODE PRE_TEST
#         DISCOVERY_TIMEOUT 10
#     )
# else()
#     gtest_discover_tests(MathLib_Unit_Tests)
# endif()
