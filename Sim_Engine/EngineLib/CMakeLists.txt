# CMakeLists.txt for EngineLib (DSFE_core)

cmake_minimum_required(VERSION 3.10)
project(DSFE_core LANGUAGES CXX)

# Define a library for GLAD from the existing source file
add_library(glad STATIC ../External/GLAD/glad.c)
target_include_directories(glad PUBLIC 
    ../External/GLAD 
    ../External # Add the parent External directory
)
find_package(OpenGL REQUIRED)

target_link_libraries(glad PRIVATE OpenGL::GL)

# Define an INTERFACE library for stb (header-only)
add_library(stb INTERFACE)
target_include_directories(stb INTERFACE ../External/stb)

# --- EngineLib Target ---
add_library(DSFE_core SHARED)

target_compile_definitions(DSFE_core PRIVATE DSFE_CORE_EXPORTS)

# Glob for all source files within the EngineLib project directory
file(GLOB_RECURSE DSFE_CORE_SOURCES 
    "src/*.cpp"
    "include/**/*.cpp"
)

# Add sources from fetched content (ImGui backends)
target_sources(DSFE_core PRIVATE
    ${DSFE_CORE_SOURCES}
    "${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp"
    "${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp"
)



# Add Windows-specific source files
if(WIN32)
    target_sources(DSFE_core PRIVATE "dllmain.cpp")
endif()

set(HDF5_ROOT ${CMAKE_SOURCE_DIR}/Sim_Engine/External/HDF5)

add_library(HDF5 STATIC IMPORTED)
set_target_properties(HDF5 PROPERTIES
    IMPORTED_LOCATION ${HDF5_ROOT}/lib/hdf5.lib
    INTERFACE_INCLUDE_DIRECTORIES ${HDF5_ROOT}/include
)

# Set public include directories for consumers of EngineLib
target_include_directories(DSFE_core PUBLIC "include")

# Set private include directories for building EngineLib
target_include_directories(DSFE_core PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}

    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/plugins
    ${imgui_SOURCE_DIR}/backends

    ${implot_SOURCE_DIR}
    ${glm_SOURCE_DIR}/glm
    ${assimp_SOURCE_DIR}/include

    ${HDF5_ROOT}/include
)

# Use precompiled headers
target_precompile_headers(DSFE_core PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/pch.h")

# Link DSFE_core against its dependencies
target_link_libraries(DSFE_core PUBLIC 
    PxMLib
    ExternalHeaders
    glfw 
    imgui 
    implot
    glad
    stb
    glm::glm
    assimp
    HDF5
)
