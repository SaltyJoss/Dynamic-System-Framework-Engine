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

# Defines the source and destination for assets and configs
set(ASSETS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/assets)
set(CONFIG_SRC_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/config)

# Add a custom command to copy the assets directory after the build
add_custom_command(TARGET DSFE_Engine POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${ASSETS_SRC_DIR}"
    "$<TARGET_FILE_DIR:DSFE_Engine>/assets"
    COMMENT "Copying assets to build directory..."
)

# Add a custom command to copy the config directory after the build
add_custom_command(TARGET DSFE_Engine POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CONFIG_SRC_DIR}"
    "$<TARGET_FILE_DIR:DSFE_Engine>/config"
    COMMENT "Copying config to build directory..."
)
