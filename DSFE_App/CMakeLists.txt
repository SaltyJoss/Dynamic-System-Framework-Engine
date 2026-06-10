# CMakeLists.txt for the DSFE_App project
cmake_minimum_required(VERSION 3.17)
project(DSFE_App LANGUAGES C CXX)

# Add projects within the DSFE_App directory
add_subdirectory(DSFE_Core)
add_subdirectory(DSFE_GUI)
add_subdirectory(DSFE_Engine)
