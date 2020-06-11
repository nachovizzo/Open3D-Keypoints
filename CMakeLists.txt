cmake_minimum_required(VERSION 3.1)
project(open3d_keypoints)

find_package(Open3D REQUIRED)
find_package(OpenMP)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g3 -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-g0 -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

link_directories(${Open3D_LIBRARY_DIRS})
include_directories(${Open3D_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/src/)

add_subdirectory(examples)
add_subdirectory(src)
add_subdirectory(tests)
