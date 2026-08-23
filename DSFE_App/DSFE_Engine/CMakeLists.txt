# CMakeLists.txt for the DSFE_Engine executable
find_package(Threads REQUIRED)

add_executable(DSFE_Engine
    src/Main.cpp
    src/BatchMain.cpp
)

target_include_directories(DSFE_Engine PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(DSFE_Engine PRIVATE 
    DSFE_Core
    Threads::Threads
)

if (DSFE_ENABLE_GUI)
    target_link_libraries(DSFE_Engine PRIVATE DSFE_GUI)
    target_compile_definitions(DSFE_Engine PRIVATE DSFE_ENABLE_GUI)
endif()

# Add post-build commands to copy runtime dependencies and assets
add_custom_command(TARGET DSFE_Engine POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
        "$<TARGET_FILE_DIR:DSFE_Engine>"

    COMMAND $<$<BOOL:$<TARGET_RUNTIME_DLLS:DSFE_Engine>>:${CMAKE_COMMAND}>
            $<$<BOOL:$<TARGET_RUNTIME_DLLS:DSFE_Engine>>:-E>
            $<$<BOOL:$<TARGET_RUNTIME_DLLS:DSFE_Engine>>:copy_if_different>
            $<$<BOOL:$<TARGET_RUNTIME_DLLS:DSFE_Engine>>:$<TARGET_RUNTIME_DLLS:DSFE_Engine>>
            $<$<BOOL:$<TARGET_RUNTIME_DLLS:DSFE_Engine>>:$<TARGET_FILE_DIR:DSFE_Engine>>

    COMMAND_EXPAND_LISTS
)

# Platform-specific configurations for Qt deployment on Windows
if(WIN32 AND DSFE_ENABLE_GUI)
    find_package(Qt6 REQUIRED COMPONENTS Core)
    get_target_property(_qmake_exe Qt6::qmake IMPORTED_LOCATION)
    get_filename_component(_qt_bin_dir "${_qmake_exe}" DIRECTORY)

    add_custom_command(TARGET DSFE_Engine POST_BUILD
        COMMAND "${_qt_bin_dir}/windeployqt.exe"
                --$<IF:$<CONFIG:Debug>,debug,release>
                --compiler-runtime
                --no-translations
                "$<TARGET_FILE:DSFE_Engine>"
        COMMENT "Deploying Qt runtime libraries for DSFE_Engine..."
    )
endif()

# Defines the source and destination for assets
set(ASSETS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/assets)

# Add a custom command to copy the assets directory after the build
add_custom_command(TARGET DSFE_Engine POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${ASSETS_SRC_DIR}"
    "$<TARGET_FILE_DIR:DSFE_Engine>/assets"
    COMMENT "Copying assets to build directory..."
)
