# CMakeLists.txt for MathLib
cmake_minimum_required(VERSION 3.17)
project(MathLib LANGUAGES CXX)

add_library(MathLib INTERFACE)

target_include_directories(MathLib INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(MathLib INTERFACE
    Eigen3::Eigen
)`