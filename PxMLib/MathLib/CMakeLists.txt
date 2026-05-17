# CMakeLists.txt for MathLib

cmake_minimum_required(VERSION 3.10)
project(MathLib LANGUAGES CXX)

add_library(MathLib STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/pch.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control/Error.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control/PID.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics/MassMatrix.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics/Nonlinear_Terms.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics/StateSpace.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/integrators/Integration.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/integrators/IntegrationAnalysis.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/kinematics/DH_Params.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/kinematics/Jacobian.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/kinematics/Transform_Utils.cpp
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