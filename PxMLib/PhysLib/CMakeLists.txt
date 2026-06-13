# CMakeLists.txt for PhysLib
cmake_minimum_required(VERSION 3.17)
project(PhysLib LANGUAGES CXX)

add_library(PhysLib STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/pch.cpp
)

target_precompile_headers(PhysLib PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/pch.h
)

target_include_directories(PhysLib
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/
)

target_link_libraries(PhysLib PUBLIC
    Eigen3::Eigen
    MathLib
)