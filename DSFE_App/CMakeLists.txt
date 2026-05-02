cmake_minimum_required(VERSION 3.10)
project(DSFE_App LANGUAGES CXX)

add_library(ExternalHeaders INTERFACE)

target_include_directories(ExternalHeaders
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/External
)

# Add projects within the DSFE_App directory
add_subdirectory(DSFE_Core)
add_subdirectory(DSFE_Engine)
add_subdirectory(DSFE_Unit_Tests)
