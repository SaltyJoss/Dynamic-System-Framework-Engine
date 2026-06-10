# CMakeLists.txt for MathLib
cmake_minimum_required(VERSION 3.17)
project(MathLib LANGUAGES CXX)

add_library(MathLib STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/pch.cpp
)

target_precompile_headers(MathLib PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/pch.h
)

target_include_directories(MathLib
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/
)

target_link_libraries(MathLib PUBLIC
    Eigen3::Eigen
)