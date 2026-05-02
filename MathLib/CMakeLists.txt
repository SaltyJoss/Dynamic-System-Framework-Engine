# CMakeLists.txt for MathLib

cmake_minimum_required(VERSION 3.10)
project(PxMLib LANGUAGES CXX)

add_library(PxMLib STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/MathLib/pch.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MathLib/src/control/Error.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MathLib/src/control/PID.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MathLib/src/dynamics/MassMatrix.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MathLib/src/dynamics/Nonlinear_Terms.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MathLib/src/dynamics/StateSpace.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MathLib/src/integrators/Integration.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MathLib/src/integrators/IntegrationAnalysis.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MathLib/src/kinematics/DH_Params.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MathLib/src/kinematics/Jacobian.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MathLib/src/kinematics/Transform_Utils.cpp
)

target_precompile_headers(PxMLib PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/MathLib/pch.h
)

target_include_directories(PxMLib 
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/MathLib/include>
        ${eigen_SOURCE_DIR}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/MathLib/
)