cmake_minimum_required(VERSION 3.16)
project(KMIDS-GFM-WRO-FE-2025 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find package for lccv
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

# Add lccv external library for camera_module
add_subdirectory(external/lccv)

# Directly add all src subfolders here
add_subdirectory(src/modules)
# add_subdirectory(src/data) add_subdirectory(src/utils)

# Add apps directory too
add_subdirectory(apps/test_temp)
add_subdirectory(apps/test_camera)
