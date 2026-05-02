# CMakeLists.txt for IntegrationTests

cmake_minimum_required(VERSION 3.10)
project(IntegrationTests LANGUAGES CXX)

enable_testing()

add_executable(IntegrationTests
    src/main.cpp
    src/AdaptiveMethodTests.cpp
    src/ConvergenceOrderTests.cpp
    src/ExplicitMethodTests.cpp
    src/ImplicitMethodTests.cpp
)

target_link_libraries(IntegrationTests PRIVATE
    DSFE_core
    gtest_main
)

include(GoogleTest)
gtest_discover_tests(IntegrationTests)
