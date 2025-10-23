cmake_minimum_required(VERSION 3.15)
project(RoboticArm_MathModelling LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

file(GLOB_RECURSE SOURCES
    src/*.cpp
    Simulation/*.cpp
    Visualisation/*.cpp
)

add_executable(RoboticArm_MathModelling ${SOURCES})

set(CMAKE_TOOLCHAIN_FILE "C:/Users/Joss/Documents/Libraries/vcpkg/scripts/buildsystems/vcpkg.cmake")
find_package(Eigen3 REQUIRED)
target_link_libraries(RoboticArm_MathModelling Eigen3::Eigen)

# Will eventually link to OpenGL here