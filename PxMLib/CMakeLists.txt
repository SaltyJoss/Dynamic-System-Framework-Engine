# CMakeLists.txt for PxMLib project
cmake_minimum_required(VERSION 3.10)
project(PxMLib LANGUAGES CXX)

# Add projects within the DSFE_App directory
add_subdirectory(MathLib)
add_subdirectory(MathLib_Unit)
