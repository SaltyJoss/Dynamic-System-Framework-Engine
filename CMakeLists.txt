# CMakeLists.txt for DSFE project (root CMakeLists.txt)
cmake_minimum_required(VERSION 3.17)
project(DSFE LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")

set(OUTPUT_BASE ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_BASE}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_BASE}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${OUTPUT_BASE}/lib)

include(FetchContent)

# --- External Dependencies ---
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 5.0.0
)
FetchContent_MakeAvailable(eigen)

FetchContent_Declare(
    pxmlib
    GIT_REPOSITORY https://github.com/SaltyJoss/PxM-Lib.git
    GIT_TAG v1.0.3
)
FetchContent_MakeAvailable(pxmlib)

FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.12.0
)
FetchContent_MakeAvailable(nlohmann_json)

option(DSFE_ENABLE_UNIT_TESTS "Build tests" OFF)
if (DSFE_ENABLE_UNIT_TESTS)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
        GIT_SHALLOW TRUE
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
    
    enable_testing()
endif()

# --- Project Subdirectories ---
add_subdirectory(DSFE_App)
