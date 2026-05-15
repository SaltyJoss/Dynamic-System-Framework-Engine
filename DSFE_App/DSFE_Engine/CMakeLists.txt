# CMakeLists.txt for the DSFE_Engine executable
cmake_minimum_required(VERSION 3.10)
project(DSFE_Engine LANGUAGES C CXX)

find_package(Threads REQUIRED)

add_executable(DSFE_Engine 
    src/Main.cpp
    src/BatchMain.cpp
)

target_include_directories(DSFE_Engine PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(DSFE_Engine PRIVATE 
    DSFE_GUI
    DSFE_Core
    Threads::Threads
)

# --- Asset Copying ---
# Defines the source and destination for assets
set(ASSETS_SRC_DIR ${CMAKE_SOURCE_DIR}/DSFE_App/DSFE_Engine/assets)
set(ASSETS_DEST_DIR $<TARGET_FILE_DIR:DSFE_Engine>/assets)

# --- Config Copying ---
# Defines the source and destination for the config file
set(CONFIG_SRC_DIR ${CMAKE_SOURCE_DIR}/DSFE_App/config)
set(CONFIG_DEST_DIR $<TARGET_FILE_DIR:DSFE_Engine>/config)

# Add a custom command to copy the assets directory after the build
add_custom_command(TARGET DSFE_Engine POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different
    "${ASSETS_SRC_DIR}"
    "${ASSETS_DEST_DIR}"
    COMMENT "Copying assets to build directory..."
)

# Add a custom command to copy the config directory after the build
add_custom_command(TARGET DSFE_Engine POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different
    "${CONFIG_SRC_DIR}"
    "${CONFIG_DEST_DIR}"
    COMMENT "Copying config to build directory..."
)
