# CMakeLists.txt for PhysLib
cmake_minimum_required(VERSION 3.17)
project(PhysLib LANGUAGES CXX)

add_library(PhysLib INTERFACE)

target_include_directories(PhysLib INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(PhysLib INTERFACE
    Eigen3::Eigen
    MathLib
)