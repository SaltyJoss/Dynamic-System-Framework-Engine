# CMakeLists.txt for DSFE_Core library
cmake_minimum_required(VERSION 3.17)
project(DSFE_Core LANGUAGES C CXX)

include(FetchContent)

set(tinyxml2_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(TINYXML2_BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    tinyxml2
    GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git
    GIT_TAGE 11.0.0
    SOURCE_SUBDIR  nonexistent-dir-to-skip-their-cmake
)
FetchContent_MakeAvailable(tinyxml2)

add_library(DSFE_Core SHARED)

# Compile tinyxml2.cpp straight into DSFE_Core — no subproject, no .pc, no tests
target_sources(DSFE_Core PRIVATE ${tinyxml2_SOURCE_DIR}/tinyxml2.cpp)
target_include_directories(DSFE_Core PRIVATE ${tinyxml2_SOURCE_DIR})

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
    "src/EngineCore.cpp"
    "src/Numerics/IntegrationService.cpp"
)

set(LOADING_SRC
    "src/Systems/RigidBodyLoader.cpp"
    "src/Systems/RigidBodyLoaderJSON.cpp"
    "src/Systems/RigidBodyLoaderURDF.cpp"
)

set(SYSTEMS_SRC
    "src/Systems/RigidBodySystem.cpp"
    "src/Systems/TrajectoryManager.cpp"
    "src/Systems/RigidBodySnapshot.cpp"
    "src/SingleBodySystem/Body.cpp"
)

set(PHYSICS_SRC
    "src/Physics/RigidBodyDynamics.cpp"
    "src/Physics/RigidBodyKinematics.cpp"
    "src/Physics/CollisionResolver.cpp"
)

set(PLATFORM_SRC
    "src/Platform/DataManager.cpp"
    "src/Platform/Logger.cpp"
    "src/Platform/Paths.cpp"
    "src/Platform/StudyRunner.cpp"
    "src/Analysis/Telemetry.cpp"
)

set(DSL_SRC
    "src/DSL/Command.cpp"
    "src/DSL/CommandContext.cpp"
    "src/DSL/CommandFactory.cpp"
    "src/DSL/Parser.cpp"
    "src/DSL/RegisterCommand.cpp"
    "src/DSL/StoredProgram.cpp"
    "src/DSL/Utils.cpp"
    "src/DSL/RunWrapper.cpp"

    "src/DSL/Commands/LoadCmd.cpp"
    "src/DSL/Commands/ParallelGroupCmd.cpp"
    "src/DSL/Commands/SetCmd.cpp"
    "src/DSL/Commands/SetVelocityCmd.cpp"
    "src/DSL/Commands/StartCmd.cpp"
    "src/DSL/Commands/StopCmd.cpp"
    "src/DSL/Commands/TrajClearCmd.cpp"
    "src/DSL/Commands/TrajSetCmd.cpp"
    "src/DSL/Commands/WaitCmd.cpp"
)

set(SIM_CORE_SRC 
    "src/Scene/SimulationCore.cpp"
)

# Add sources for DSFE_Core, including ImGui files
target_sources(DSFE_Core PRIVATE
    ${DEP_CORE_SRC}
    ${SYSTEMS_SRC}
    ${LOADING_SRC}
    ${PHYSICS_SRC}
    ${PLATFORM_SRC}
    ${DSL_SRC}
    ${SIM_CORE_SRC}
)

# Add Windows-specific source files
if(WIN32)
    target_sources(DSFE_Core PRIVATE "dllmain.cpp")
endif()

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

# Set include directories for building DSFE_Core
target_include_directories(DSFE_Core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
