add_library(MathLib STATIC)

# Find all source files in the src directory and add them to the target
file(GLOB_RECURSE MATHLIB_SOURCES "src/*.cpp")
target_sources(MathLib PRIVATE ${MATHLIB_SOURCES})

target_include_directories(MathLib 
    PUBLIC 
        # Expose the 'include' directory to other targets that link against MathLib
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        # Add the 'src' directory for MathLib's own implementation files
        src
)

# Link to Eigen, which is provided by FetchContent in the root CMakeLists.txt
# This will also handle including Eigen's headers.
target_link_libraries(MathLib PUBLIC Eigen::eigen)
