# CMakeLists.txt for MathLib_Unit_Tests executable
cmake_minimum_required(VERSION 3.17)
project(PhysLib_SingleBodySystemTests LANGUAGES C CXX)

enable_testing()

add_executable(PhysLib_SingleBodySystemTests
    main.cpp
    SingleBodySystemDynamicsTests.cpp
)

target_link_libraries(PhysLib_SingleBodySystemTests PRIVATE
    PhysLib
    gtest_main
)

target_include_directories(PhysLib_SingleBodySystemTests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Common
)

include(GoogleTest)

gtest_discover_tests(
    PhysLib_SingleBodySystemTests
    DISCOVERY_MODE PRE_TEST
    DISCOVERY_TIMEOUT 10
)
