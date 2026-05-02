# CMakeLists.txt for the Engine executable

cmake_minimum_required(VERSION 3.10)
project(DSFE_app LANGUAGES CXX)

add_executable(DSFE_app
    src/Main.cpp
    src/BatchMain.cpp
)

target_include_directories(DSFE_app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(DSFE_app PRIVATE 
    DSFE_core
)

# --- Asset Copying ---
# Define the source and destination for assets
set(ASSETS_SRC_DIR ${CMAKE_SOURCE_DIR}/Sim_Engine/Engine/assets)
set(ASSETS_DEST_DIR $<TARGET_FILE_DIR:DSFE_app>/assets)

# Add a custom command to copy the assets directory after the build
add_custom_command(TARGET DSFE_app POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different
    "${ASSETS_SRC_DIR}"
    "${ASSETS_DEST_DIR}"
    COMMENT "Copying assets to build directory..."
)
