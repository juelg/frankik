cmake_minimum_required(VERSION 3.5)

project(
    frankik
    LANGUAGES C CXX
    VERSION 0.1
    DESCRIPTION "Blazing fast, analytical Inverse Kinematics for Franka Emika Panda and FR3 robots. Lightweight Python bindings, no ROS required."
)


cmake_policy(SET CMP0048 NEW) # Set version in project
# Allow target properties affecting visibility during linking in static libraries
set(CMAKE_POLICY_DEFAULT_CMP0063 NEW)
cmake_policy(SET CMP0135 NEW) # Use timestamp of file extraction not download
cmake_policy(SET CMP0140 NEW) # Check return arguments

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(BUILD_SHARED_LIBS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(FetchContent)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)


FetchContent_Declare(pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG v2.13.6
    GIT_PROGRESS TRUE
    EXCLUDE_FROM_ALL
)
FetchContent_Declare(Eigen3
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.1
    GIT_PROGRESS TRUE
    EXCLUDE_FROM_ALL
)

FetchContent_MakeAvailable(pybind11 Eigen3)

add_subdirectory(src)
