# Define a library for GLAD from the existing source file
add_library(glad STATIC ../External/GLAD/glad.c)
target_include_directories(glad PUBLIC 
    ../External/GLAD 
    ../External # Add the parent External directory
)
find_package(OpenGL REQUIRED)
target_link_libraries(glad PRIVATE OpenGL::GL glfw)

# Define an INTERFACE library for stb (header-only)
add_library(stb INTERFACE)
target_include_directories(stb INTERFACE ../External/stb)

# --- EngineLib Target ---
add_library(EngineLib SHARED)

# Glob for all source files within the EngineLib project directory
file(GLOB_RECURSE ENGINELIB_SOURCES 
    "src/*.cpp"
    "include/**/*.cpp"
)

# Add sources from fetched content (ImGui backends)
target_sources(EngineLib PRIVATE
    ${ENGINELIB_SOURCES}
    "${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp"
    "${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp"
)

# Add Windows-specific source files
if(WIN32)
    target_sources(EngineLib PRIVATE "dllmain.cpp")
endif()

# Set public include directories for consumers of EngineLib
target_include_directories(EngineLib PUBLIC 
    "include"
)

# Set private include directories for building EngineLib
target_include_directories(EngineLib PRIVATE
    # Add ImGui and ImPlot include directories
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${implot_SOURCE_DIR}
    ${glm_SOURCE_DIR}/glm # For glm
    ${assimp_SOURCE_DIR}/include # For assimp
)

# Use precompiled headers
target_precompile_headers(EngineLib PRIVATE pch.h)

# Link EngineLib against its dependencies
target_link_libraries(EngineLib PUBLIC 
    MathLib 
    glfw 
    imgui 
    implot
    glad
    stb
    glm
    assimp::assimp
)
