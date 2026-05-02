cmake_minimum_required(VERSION 3.10)
project(DSFE_main LANGUAGES CXX)

add_library(ExternalHeaders INTERFACE)

target_include_directories(ExternalHeaders
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/External
)

# Add projects within the Sim_Engine directory
add_subdirectory(EngineLib)
add_subdirectory(Engine)
add_subdirectory(IntegrationTests)
