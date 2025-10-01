cmake_minimum_required(VERSION 3.16)
project(KMIDS-GFM-WRO-FE-2025 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find package for lccv
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA libcamera)

# Add lccv external library for camera_module
if(LIBCAMERA_FOUND)
  add_subdirectory(external/lccv)
endif()

add_subdirectory(src/utils)
add_subdirectory(src/modules)
add_subdirectory(src/processors)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_subdirectory(apps/test_camera)
add_subdirectory(apps/test_lidar)
add_subdirectory(apps/test_i2c)
add_subdirectory(apps/test_lidar_cam)
add_subdirectory(apps/log_viewer)
add_subdirectory(apps/scan_map_outer)
add_subdirectory(apps/scan_map_inner)
add_subdirectory(apps/challenges/open_challenge)
add_subdirectory(apps/challenges/obstacle_challenge)
