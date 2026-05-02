# CMakeLists.txt for EngineLib (DSFE_Core)

cmake_minimum_required(VERSION 3.10)
project(DSFE_Core LANGUAGES CXX)

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

# DSFE_Core library definition
add_library(DSFE_Core SHARED)

target_compile_definitions(DSFE_Core PRIVATE 
    DSFE_CORE_EXPORTS
    IMGUI_ENABLE_DOCKING
)

set(CORE_SRC
    src/Application.cpp
    src/EngineCore.cpp

    src/Numerics/IntegrationService.cpp

    src/Physics/PhysicsSystem.cpp

    src/Robots/RobotDynamics.cpp
    src/Robots/RobotKinematics.cpp
    src/Robots/RobotLoader.cpp
    src/Robots/RobotModel.cpp
    src/Robots/RobotSystem.cpp
    src/Robots/TrajectoryManager.cpp

    src/Interpreter/Command.cpp
    src/Interpreter/CommandContextMotion.cpp
    src/Interpreter/CommandFactory.cpp
    src/Interpreter/MainContext.cpp
    src/Interpreter/Parser.cpp
    src/Interpreter/RegisterCommand.cpp
    src/Interpreter/StoredProgram.cpp
    src/Interpreter/UIContext.cpp
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
    src/Platform/WindowManager.cpp

    src/Assets/MeshLoader.cpp
    src/Assets/importObj.cpp
    src/Assets/std_image_impl.cpp

    src/Analysis/Telemetry.cpp
)

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
    src/Scene/SimulationCore.cpp
    src/Scene/SimulationManager.cpp
)

set(UI_SRC
    src/ui/CommandScriptEditor.cpp
    src/ui/ControlPanel.cpp
    src/ui/DebugPanel.cpp
    src/ui/RenderPreset.cpp
    src/ui/Styles.cpp
)

# Add sources for DSFE_Core, including ImGui files
target_sources(DSFE_Core PRIVATE
    ${CORE_SRC}
    ${PLATFORM_SRC}
    ${RENDER_SRC}
    ${SCENE_SRC}
    ${UI_SRC}

    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

# Add Windows-specific source files
if(WIN32)
    target_sources(DSFE_Core PRIVATE "dllmain.cpp")
endif()

set(HDF5_ROOT ${CMAKE_SOURCE_DIR}/DSFE_App/External/HDF5)
set(imgui_ROOT ${CMAKE_SOURCE_DIR}/DSFE_App/External/imgui)

add_library(implot STATIC
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
)

target_include_directories(implot PUBLIC
    ${implot_SOURCE_DIR}
    ${imgui_SOURCE_DIR}
)

add_library(HDF5 INTERFACE)

target_include_directories(HDF5 INTERFACE
    ${HDF5_ROOT}/include
)

target_compile_definitions(HDF5 INTERFACE
    H5_BUILT_AS_DYNAMIC_LIB
)

target_link_libraries(HDF5 INTERFACE
    $<$<CONFIG:Debug>:${HDF5_ROOT}/lib/hdf5.lib>
    $<$<CONFIG:Release>:${HDF5_ROOT}/lib/hdf5.lib>
    $<$<CONFIG:RelWithDebInfo>:${HDF5_ROOT}/lib/hdf5.lib>
    $<$<CONFIG:MinSizeRel>:${HDF5_ROOT}/lib/hdf5.lib>
)

# Set public include directories for consumers of EngineLib
target_include_directories(DSFE_Core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Set private include directories for building EngineLib
target_include_directories(DSFE_Core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include

    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${imgui_ROOT}/plugins

    ${implot_SOURCE_DIR}
    ${glm_SOURCE_DIR}/glm
    ${assimp_SOURCE_DIR}/include

    ${HDF5_ROOT}/include
)

# Use precompiled headers
target_precompile_headers(DSFE_Core PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/pch.h")

# Link DSFE_Core against its dependencies
target_link_libraries(DSFE_Core PUBLIC 
    PxMLib
    ExternalHeaders
    glfw
    implot
    glad
    stb
    glm::glm
    assimp
    HDF5
)
