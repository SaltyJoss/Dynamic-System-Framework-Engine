add_executable(Engine src/Main.cpp)

target_link_libraries(Engine PRIVATE EngineLib)

# --- Asset Copying ---
# Define the source and destination for assets
set(ASSETS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../assets)
set(ASSETS_DEST_DIR $<TARGET_FILE_DIR:Engine>/assets)

# Add a custom command to copy the assets directory after the build
add_custom_command(TARGET Engine POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different
    "${ASSETS_SRC_DIR}"
    "${ASSETS_DEST_DIR}"
    COMMENT "Copying assets to build directory..."
)
