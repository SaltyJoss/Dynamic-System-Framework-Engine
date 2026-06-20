# CMakeLists.txt for DSFE_Core library
cmake_minimum_required(VERSION 3.17)
project(DSFE_Core LANGUAGES C CXX)

add_library(DSFE_Core SHARED)

find_package(Threads REQUIRED)
find_package(HDF5 REQUIRED)

target_compile_features(DSFE_Core PUBLIC cxx_std_23)
if (MSVC)
    target_compile_options(DSFE_Core PRIVATE /bigobj)
else()
    target_compile_options(DSFE_Core PRIVATE -Wall -Wextra -Wpedantic)
endif()
target_compile_definitions(DSFE_Core PRIVATE DSFE_CORE_EXPORTS)

target_compile_definitions(DSFE_Core PUBLIC
    $<$<PLATFORM_ID:Windows>:DSFE_PLATFORM_WINDOWS>
    $<$<PLATFORM_ID:Linux>:DSFE_PLATFORM_LINUX>
)

if (NOT MSVC)
    set_target_properties(DSFE_Core PROPERTIES
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN YES
    )
endif()

set(DEP_CORE_SRC
    src/EngineCore.cpp
    src/Numerics/IntegrationService.cpp
)

set(SINGLE_BODY_SYS_SRC
    src/SingleBodySystem/Body.cpp
)

set(MULTI_BODY_SYS_SRC
    src/Robots/RobotDynamics.cpp
    src/Robots/RobotKinematics.cpp
    src/Robots/RobotLoader.cpp
    src/Robots/RobotSystem.cpp
    src/Robots/TrajectoryManager.cpp
    src/Robots/RobotSimSnapshot.cpp
)

set(PLATFORM_SRC
    src/Platform/DataManager.cpp
    src/Platform/Logger.cpp
    src/Platform/Paths.cpp
    src/Platform/StudyRunner.cpp
    src/Analysis/Telemetry.cpp
)

set(DSL_SRC
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

set(SIM_CORE_SRC src/Scene/SimulationCore.cpp)

# Add sources for DSFE_Core, including ImGui files
target_sources(DSFE_Core PRIVATE
    ${DEP_CORE_SRC}
    ${SINGLE_BODY_SYS_SRC}
    ${MULTI_BODY_SYS_SRC}
    ${PLATFORM_SRC}
    ${DSL_SRC}
    ${SIM_CORE_SRC}
)

# Add Windows-specific source files
if(WIN32)
    target_sources(DSFE_Core PRIVATE "dllmain.cpp")
endif()

# Set include directories for building DSFE_Core
target_include_directories(DSFE_Core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Use precompiled headers
target_precompile_headers(DSFE_Core PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/pch.h")

# Link DSFE_Core against its dependencies
target_link_libraries(DSFE_Core
    PUBLIC
        Eigen3::Eigen
        MathLib
        PhysLib
        Threads::Threads

    PRIVATE
        nlohmann_json::nlohmann_json
)

if (TARGET hdf5::hdf5-shared)
    target_link_libraries(DSFE_Core PRIVATE hdf5::hdf5-shared)
elseif (TARGET HDF5::HDF5)
    target_link_libraries(DSFE_Core PRIVATE HDF5::HDF5)
else()
    target_link_libraries(DSFE_Core PRIVATE ${HDF5_LIBRARIES})
endif()
