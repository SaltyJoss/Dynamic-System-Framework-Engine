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
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)

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

set(ASSETS_SRC src/Assets/std_image_impl.cpp)

target_sources(DSFE_GUI PRIVATE
    ${RENDER_SRC}
    ${SCENE_SRC}
    ${UI_SRC}
    ${PLATFORM_SRC}
    ${ASSETS_SRC}

    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

set(imgui_ROOT ${CMAKE_SOURCE_DIR}/DSFE_App/External/imgui)

add_library(implot STATIC
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
)

target_include_directories(implot PUBLIC
    ${implot_SOURCE_DIR}
    ${imgui_SOURCE_DIR}
)

target_include_directories(DSFE_Core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include

    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${imgui_ROOT}/plugins

    ${implot_SOURCE_DIR}
    ${glm_SOURCE_DIR}/glm
    ${assimp_SOURCE_DIR}/include
)

# Link DSFE_Core against its dependencies
target_link_libraries(DSFE_GUI PUBLIC
    DSFE_Core
    ExternalHeaders
    implot
    stb
    assimp
    OpenGL::GL
    Threads::Threads
)

target_link_libraries(DSFE_GUI PRIVATE
    glfw
    glad
    glm::glm
)