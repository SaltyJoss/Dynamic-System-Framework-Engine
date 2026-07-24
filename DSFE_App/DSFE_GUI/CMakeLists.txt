# CMakeLists.txt for DSFE_GUI module
cmake_minimum_required(VERSION 3.17)
project(DSFE_GUI LANGUAGES C CXX)

message(CONFIGURE_LOG "Configuring the Vulkan renderer backend for DSFE GUI.")

include(FetchContent)

# Fetch GLM from GitHub
FetchContent_Declare(
  glm
  GIT_REPOSITORY https://github.com/g-truc/glm.git
  GIT_TAG 1.0.3
)
set(GLM_BUILD_LIBRARY OFF CACHE BOOL "" FORCE)
set(GLM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glm)

FetchContent_Declare(
    assimp
    GIT_REPOSITORY https://github.com/assimp/assimp.git
    GIT_TAG  v6.0.4
)
# Core build hygiene
set(ASSIMP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_ASSIMP_TOOLS OFF CACHE BOOL "" FORCE)
set(ASSIMP_NO_EXPORT ON CACHE BOOL "" FORCE)
set(ASSIMP_WARNINGS_AS_ERRORS OFF CACHE BOOL "" FORCE)
# ZIP / ZLIB control (important for linux builds)
set(ASSIMP_BUILD_ZLIB ON CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_NO_OWN_ZLIB OFF CACHE BOOL "" FORCE)
set(ASSIMP_USE_SYSTEM_ZLIB OFF CACHE BOOL "" FORCE)
# IMPORTANT: prevents system detection overriding FetchContent intent
set(ZLIB_FOUND OFF CACHE BOOL "" FORCE)
# Optional importers/exporters
set(ASSIMP_BUILD_USD_IMPORTER OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_M3D_IMPORTER OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_M3D_EXPORTER OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_C4D_IMPORTER OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_VRML_IMPORTER OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(assimp)

FetchContent_Declare(
    vma
    GIT_REPOSITORY https://github.com/GPUOpen-LibrariesAndSDKs/VulkanMemoryAllocator.git
    GIT_TAG v3.2.1
)
FetchContent_MakeAvailable(vma)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
#set(CMAKE_PREFIX_PATH "C:/Qt/6.11.1/msvc2022_64")

add_library(DSFE_GUI SHARED)

find_package(Threads REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS
    Core
    Gui
    Widgets
)

target_compile_definitions(DSFE_GUI PRIVATE
    DSFE_GUI_EXPORTS
    GLM_ENABLE_EXPERIMENTAL
    GLM_FORCE_DEPTH_ZERO_TO_ONE
    VK_NO_PROTOTYPES
    VMA_STATIC_VULKAN_FUNCTIONS=0
    VMA_DYNAMIC_VULKAN_FUNCTIONS=1
    $<$<PLATFORM_ID:Windows>:VK_USE_PLATFORM_WIN32_KHR>
    $<$<PLATFORM_ID:Windows>:NOMINMAX>
    $<$<PLATFORM_ID:Windows>:WIN32_LEAN_AND_MEAN>
    $<$<PLATFORM_ID:Linux>:VK_USE_PLATFORM_XCB_KHR>
    $<$<PLATFORM_ID:Linux>:VK_USE_PLATFORM_WAYLAND_KHR>
)

add_library(stb INTERFACE)

target_include_directories(stb INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(MAIN_WINDOW_SRC
    src/MainWindow/DSFE_MainWindow.cpp
    src/MainWindow/Workspace/ProjectPage.cpp
    src/MainWindow/Workspace/Workspace.cpp
    src/MainWindow/style/DSFETheme.cpp
)

set(SCENE_SRC
    src/Scene/Camera.cpp
    src/Scene/Mesh.cpp
    src/Scene/Object.cpp
)

set(SIMULATION_SRC
    src/Simulation/MeshStore.cpp
    src/Simulation/SimulationManager.cpp
    src/Simulation/SimulationScene.cpp
    src/Simulation/SimulationRenderer.cpp
    src/Simulation/SimulationSystemController.cpp
)

set(SYSTEMS_SRC
    src/Systems/SingleBodySystem.cpp
    src/Systems/MultiBodySystem.cpp
)

set(WIDGETS_SRC
    src/ui/RenderPreset.cpp
    src/MainWindow/Widgets/ViewportWidget.cpp
    include/MainWindow/Widgets/ViewportWidget.h
    src/MainWindow/Widgets/RobotSelectorWidget.cpp
    include/MainWindow/Widgets/RobotSelectorWidget.h
    src/MainWindow/Widgets/ControlPanelWidget.cpp
    include/MainWindow/Widgets/ControlPanelWidget.h
    src/MainWindow/Widgets/DSLEditorWidget.cpp
    include/MainWindow/Widgets/DSLEditorWidget.h
    src/MainWindow/Widgets/ConsoleOutputWidget.cpp
    include/MainWindow/Widgets/ConsoleOutputWidget.h
    src/MainWindow/Widgets/FractionSelectorWidget.cpp
    include/MainWindow/Widgets/FractionSelectorWidget.h
    src/MainWindow/DSL/DSLSyntaxHighlighter.cpp
    include/MainWindow/DSL/DSLSyntaxHighlighter.h
)

set(ROBOT_SRC
    src/Robots/RobotPresentationBuilder.cpp
    src/Robots/RobotRenderer.cpp
)

set(OBJECT_SRC
    src/Assets/MeshPresentationBuilder.cpp
)

set(PLATFORM_SRC 
    src/Application.cpp
)

set(ASSETS_SRC 
    src/Assets/std_image_impl.cpp
    src/Assets/MeshLoader.cpp
    src/Assets/importObj.cpp
)

set(RENDERER_SRC
    src/Renderer/vk_mem_alloc_impl.cpp
    src/Renderer/VulkanContext.cpp
    src/Renderer/VulkanLoader.cpp
    src/Renderer/VulkanRenderer.cpp
    src/Renderer/VulkanSwapchain.cpp
)

set(QT_RESOURCES 
    resources/DSFE_GUI.qrc
)

target_sources(DSFE_GUI PRIVATE
    ${MAIN_WINDOW_SRC}
    ${SCENE_SRC}
    ${SIMULATION_SRC}
    ${SYSTEMS_SRC}
    ${WIDGETS_SRC}
    ${ROBOT_SRC}
    ${OBJECT_SRC}
    ${PLATFORM_SRC}
    ${ASSETS_SRC}
    ${RENDERER_SRC}
    ${QT_RESOURCES}
)

find_package(Vulkan REQUIRED COMPONENTS volk shaderc_combined)

target_include_directories(DSFE_GUI
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/MainWindow
        ${CMAKE_CURRENT_SOURCE_DIR}/include/MainWindow/style
)

# Platform-specific configurations
if(WIN32)
    target_include_directories(DSFE_GUI PRIVATE "${Vulkan_INCLUDE_DIR}/Volk")
elseif(UNIX AND NOT APPLE)
    find_path(VOLK_INCLUDE_DIR volk.h PATH_SUFFIXES Volk volk)
    if(NOT VOLK_INCLUDE_DIR)
        message(FATAL_ERROR "Volk include directory not found")
    endif()
    target_include_directories(DSFE_GUI PRIVATE ${VOLK_INCLUDE_DIR})
endif()

# Link against XCB on Linux for Vulkan surface creation
if(UNIX AND NOT APPLE)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(XCB REQUIRED xcb)
    target_include_directories(DSFE_GUI PRIVATE ${XCB_INCLUDE_DIRS})
    target_link_libraries(DSFE_GUI PRIVATE ${XCB_LIBRARIES})
endif()

# Link DSFE_Core against its dependencies
target_link_libraries(DSFE_GUI
    PUBLIC
        DSFE_Core
        Eigen3::Eigen
        Threads::Threads

    PRIVATE
        stb
        assimp
        glm::glm
        Vulkan::Headers
        Vulkan::volk
        Vulkan::shaderc_combined
        GPUOpen::VulkanMemoryAllocator
        Qt6::Core
        Qt6::Gui
        Qt6::Widgets
)

# Defines the source and destination for assets and configs
set(ASSETS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/assets)

# Add a custom command to copy the assets directory after the build
add_custom_command(TARGET DSFE_GUI POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
        "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/$<CONFIG>/assets"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${ASSETS_SRC_DIR}"
        "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/$<CONFIG>/assets"
)