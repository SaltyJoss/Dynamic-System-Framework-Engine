cmake_minimum_required(VERSION 3.14)
project(SimEngine)

# Add projects within the Sim_Engine directory
add_subdirectory(EngineLib)
add_subdirectory(Engine)
add_subdirectory(IntegrationTests)
