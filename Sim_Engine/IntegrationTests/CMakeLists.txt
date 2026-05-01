enable_testing()

add_executable(IntegrationTests
    src/main.cpp
    src/AdaptiveMethodTests.cpp
    src/ConvergenceOrderTests.cpp
    src/ExplicitMethodTests.cpp
    src/ImplicitMethodTests.cpp
)

target_link_libraries(IntegrationTests PRIVATE
    EngineLib
    gtest_main
)

include(GoogleTest)
gtest_discover_tests(IntegrationTests)
