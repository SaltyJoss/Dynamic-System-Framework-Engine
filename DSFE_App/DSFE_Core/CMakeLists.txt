# CMakeLists.txt for DSFE_Core library
cmake_minimum_required(VERSION 3.10)
project(DSFE_Core LANGUAGES C CXX)

add_library(DSFE_Core SHARED)

find_package(Threads REQUIRED)
find_package(HDF5 CONFIG REQUIRED)

target_compile_definitions(DSFE_Core PRIVATE DSFE_CORE_EXPORTS)

set(CORE_SRC
    src/EngineCore.cpp

    src/Numerics/IntegrationService.cpp

    src/Robots/RobotDynamics.cpp
    src/Robots/RobotKinematics.cpp
    src/Robots/RobotLoader.cpp
    src/Robots/RobotSystem.cpp
    src/Robots/TrajectoryManager.cpp

    src/Interpreter/Command.cpp
    src/Interpreter/CommandContext.cpp
    src/Interpreter/CommandFactory.cpp
    src/Interpreter/Parser.cpp
    src/Interpreter/RegisterCommand.cpp
    src/Interpreter/StoredProgram.cpp
    src/Interpreter/Utils.cpp
    src/Interpreter/RunWrapper.cpp

    src/Interpreter/Commands/LoadCmd.cpp
    src/Interpreter/Commands/ParallelGroupCmd.cpp
    src/Interpreter/Commands/RotateByCmd.cpp
    src/Interpreter/Commands/RotateJointByCmd.cpp
    src/Interpreter/Commands/RotateJointToCmd.cpp
    src/Interpreter/Commands/RotateToCmd.cpp
    src/Interpreter/Commands/SelectCmd.cpp
    src/Interpreter/Commands/SetCmd.cpp
    src/Interpreter/Commands/SetOmegaCmd.cpp
    src/Interpreter/Commands/SpinCmd.cpp
    src/Interpreter/Commands/StartCmd.cpp
    src/Interpreter/Commands/StopCmd.cpp
    src/Interpreter/Commands/TrajClearCmd.cpp
    src/Interpreter/Commands/TrajSetCmd.cpp
    src/Interpreter/Commands/WaitCmd.cpp
)

set(PLATFORM_SRC
    src/Platform/DataManager.cpp
    src/Platform/Logger.cpp
    src/Platform/Paths.cpp
    src/Platform/StudyRunner.cpp

    src/Analysis/Telemetry.cpp
)

set(SCENE_SRC src/Scene/SimulationCore.cpp)

# Add sources for DSFE_Core, including ImGui files
target_sources(DSFE_Core PRIVATE
    ${CORE_SRC}
    ${PLATFORM_SRC}
    ${SCENE_SRC}
)

# Add Windows-specific source files
if(WIN32)
    target_sources(DSFE_Core PRIVATE "dllmain.cpp")
endif()

# Set include directories for building DSFE_Core
target_include_directories(DSFE_Core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include

    ${HDF5_INCLUDE_DIRS}
)

# Use precompiled headers
target_precompile_headers(DSFE_Core PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/pch.h")

# Link DSFE_Core against its dependencies
target_link_libraries(DSFE_Core
    PUBLIC 
        PxMLib
        Eigen3::Eigen
        hdf5::hdf5-shared
        Threads::Threads

    PRIVATE
        assimp
        nlohmann_json::nlohmann_json
)
