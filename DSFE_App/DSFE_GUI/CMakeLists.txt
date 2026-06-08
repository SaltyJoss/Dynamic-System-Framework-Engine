# CMakeLists.txt for DSFE_GUI module
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

cmake_minimum_required(VERSION 3.10)
project(DSFE_GUI LANGUAGES C CXX)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_PREFIX_PATH "C:/Qt/6.11.1/msvc2022_64")

add_library(DSFE_GUI SHARED)

find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS
    Core
    Gui
    Widgets
    OpenGLWidgets
)

target_compile_definitions(DSFE_GUI PRIVATE
    DSFE_GUI_EXPORTS
    IMGUI_ENABLE_DOCKING
    GLM_ENABLE_EXPERIMENTAL
)

# Define a library for GLAD from the existing source file
add_library(glad STATIC ../External/GLAD/glad.c)
target_include_directories(glad PUBLIC ../External/GLAD)
target_link_libraries(glad PUBLIC OpenGL::GL)

add_library(stb INTERFACE)

target_include_directories(stb INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(MAIN_WINDOW_SRC
    src/MainWindow/DSFE_MainWindow.cpp
    src/MainWindow/Workspace/ProjectPage.cpp
    src/MainWindow/style/DSFETheme.cpp
)

set(RENDER_SRC
    src/Rendering/CubeVertices.cpp
    src/Rendering/IBL.cpp
    src/Rendering/OpenGLBufferManager.cpp
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

set(MANAGER_SRC
    src/Scene/Manager/SimBackend.cpp
    src/Scene/Manager/SimCamera.cpp
    src/Scene/Manager/SimObjects.cpp
    src/Scene/Manager/SimRendering.cpp
    src/Scene/Manager/SimRobots.cpp
    src/Scene/Manager/SimUI.cpp
    src/Scene/Manager/SimViewport.cpp
)

set(WIDGETS_SRC
    src/ui/RenderPreset.cpp
    src/MainWindow/Widgets/ViewportWidget.cpp
    src/MainWindow/Widgets/RobotSelectorWidget.cpp
    src/MainWindow/Widgets/ControlPanelWidget.cpp
    src/MainWindow/Widgets/DSLEditorWidget.cpp
    src/MainWindow/Widgets/ConsoleOutputWidget.cpp
    src/MainWindow/Widgets/FractionSelectorWidget.cpp
    include/MainWindow/Widgets/FractionSelectorWidget.h
    src/MainWindow/DSL/DSLSyntaxHighlighter.cpp
)

set(ROBOT_SRC
    src/Robots/RobotPresentationBuilder.cpp
    src/Robots/RobotRenderer.cpp
)

set(PLATFORM_SRC 
    src/Application.cpp
)

set(ASSETS_SRC 
    src/Assets/std_image_impl.cpp
    src/Assets/MeshLoader.cpp
    src/Assets/importObj.cpp
)

set(QT_RESOURCES 
    resources/DSFE_GUI.qrc
)

target_sources(DSFE_GUI PRIVATE
    ${MAIN_WINDOW_SRC}
    ${RENDER_SRC}
    ${SCENE_SRC}
    ${MANAGER_SRC}
    ${WIDGETS_SRC}
    ${ROBOT_SRC}
    ${PLATFORM_SRC}
    ${ASSETS_SRC}
    ${QT_RESOURCES}
)

set(imfilebrowser_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/include/imfilebrowser)

add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp

    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

target_include_directories(imgui PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)

target_link_libraries(imgui PUBLIC glfw OpenGL::GL)

add_library(implot STATIC
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
)

target_include_directories(implot PUBLIC
    ${implot_SOURCE_DIR}
    ${imgui_SOURCE_DIR}
)

target_include_directories(DSFE_GUI
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/MainWindow
        ${CMAKE_CURRENT_SOURCE_DIR}/include/Scene
        ${CMAKE_CURRENT_SOURCE_DIR}/include/MainWindow/style

    PRIVATE
        ${imfilebrowser_ROOT}
)

# Link DSFE_Core against its dependencies
target_link_libraries(DSFE_GUI
    PUBLIC
        DSFE_Core
        Eigen3::Eigen
        Threads::Threads

    PRIVATE
        implot
        stb
        assimp
        glfw
        glad
        glm::glm
        imgui
        OpenGL::GL
        Qt6::Core
        Qt6::Gui
        Qt6::Widgets
        Qt6::OpenGLWidgets
)