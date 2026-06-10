# CMakeLists.txt for MathLib_Unit_Tests executable
cmake_minimum_required(VERSION 3.17)
project(MathLib_ADTests LANGUAGES C CXX)

enable_testing()

add_executable(MathLib_ADTests
    main.cpp
    ADNewtonTests.cpp
    ADJacobianTests.cpp
    ADExplicitIntegratorTests.cpp
    ADImplicitIntegratorTests.cpp
)

target_link_libraries(MathLib_ADTests PRIVATE
    MathLib
    gtest_main
)

target_include_directories(MathLib_ADTests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Common
)

include(GoogleTest)

gtest_discover_tests(
    MathLib_ADTests
    DISCOVERY_MODE PRE_TEST
    DISCOVERY_TIMEOUT 10
)

# if (CMAKE_BUILD_TYPE STREQUAL "Debug")
#     gtest_discover_tests(
#         MathLib_DualNumberTests
#         DISCOVERY_MODE PRE_TEST
#         DISCOVERY_TIMEOUT 10
#     )
# else()
#     gtest_discover_tests(MathLib_DualNumberTests)
# endif()
