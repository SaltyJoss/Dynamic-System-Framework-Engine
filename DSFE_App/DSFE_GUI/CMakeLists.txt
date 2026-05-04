# CMakeLists.txt for DSFE_GUI module
cmake_minimum_required(VERSION 3.10)
project(DSFE_GUI LANGUAGES C CXX)

add_library(DSFE_GUI SHARED)

find_package(Threads REQUIRED)

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

target_compile_definitions(DSFE_GUI PRIVATE IMGUI_ENABLE_DOCKING)

set(RENDER_SRC
    src/Rendering/CubeVertices.cpp
    src/Rendering/GUIContext.cpp
    src/Rendering/IBL.cpp
    src/Rendering/OpenGLBufferManager.cpp
    src/Rendering/OpenGLContext.cpp
    src/Rendering/ShaderUtil.cpp
    src/Rendering/SkyboxRenderer.cpp
    src/Rendering/Texture.cpp
)

set(SCENE_SRC
    src/Scene/AxisOrientator.cpp
    src/Scene/Camera.cpp
    src/Scene/Input.cpp
    src/Scene/Mesh.cpp
    src/Scene/Object.cpp
    src/Scene/SimulationManager.cpp
)

set(UI_SRC
    src/ui/CommandScriptEditor.cpp
    src/ui/ControlPanel.cpp
    src/ui/DebugPanel.cpp
    src/ui/RenderPreset.cpp
    src/ui/Styles.cpp
)

set(ROBOT_SRC
    src/Robots/RobotPresentationBuilder.cpp
)

set(PLATFORM_SRC src/Platform/WindowManager.cpp)

set(ASSETS_SRC 
    src/Assets/std_image_impl.cpp
    src/Assets/MeshLoader.cpp
    src/Assets/importObj.cpp
)

target_sources(DSFE_GUI PRIVATE
    ${RENDER_SRC}
    ${SCENE_SRC}
    ${UI_SRC}
    ${PLATFORM_SRC}
    ${ASSETS_SRC}
)

set(imfilebrowser_ROOT ${CMAKE_SOURCE_DIR}/DSFE_App/External/imgui/plugins)

target_include_directories(DSFE_GUI PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include

    ${imgui_SOURCE_DIR}/backends
    ${imfilebrowser_ROOT}
)

# Link DSFE_Core against its dependencies
target_link_libraries(DSFE_GUI
    PUBLIC
        DSFE_Core

    PRIVATE
        ExternalHeaders
        implot
        stb
        assimp
        glfw
        glad
        glm
        imgui
        OpenGL::GL
        Threads::Threads
)