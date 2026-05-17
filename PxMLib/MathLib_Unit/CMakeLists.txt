# CMakeLists.txt for MathLib_Unit_Tests executable
cmake_minimum_required(VERSION 3.10)
project(MathLib_Unit_Tests LANGUAGES C CXX)

add_subdirectory(IntegratorTests)
add_subdirectory(DualNumberTests)
add_subdirectory(AutomaticDifferentiationTests)