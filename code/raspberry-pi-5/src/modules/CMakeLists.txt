# NOTE: lidar_module

# Building rplidar_sdk
set(RPLIDAR_SDK_PATH ${CMAKE_SOURCE_DIR}/external/rplidar_sdk/sdk)
include_directories(${RPLIDAR_SDK_PATH}/include)
include_directories(${RPLIDAR_SDK_PATH}/src)
file(GLOB_RECURSE RPLIDAR_SDK_SOURCES ${RPLIDAR_SDK_PATH}/src/*.cpp)
list(FILTER RPLIDAR_SDK_SOURCES EXCLUDE REGEX ".*macOS.*")
list(FILTER RPLIDAR_SDK_SOURCES EXCLUDE REGEX ".*win32.*")
add_library(rplidar_sdk STATIC ${RPLIDAR_SDK_SOURCES})

add_library(lidar_module STATIC lidar_module.cpp lidar_module.h)
target_include_directories(
  lidar_module
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
         ${CMAKE_SOURCE_DIR}/external/rplidar_sdk/sdk/include
         ${CMAKE_SOURCE_DIR}/src/types)
target_link_libraries(lidar_module PRIVATE rplidar_sdk)

# NOTE: camera_module

add_library(camera_module STATIC camera_module.cpp camera_module.h)
target_include_directories(
  camera_module
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS}
         ${LIBCAMERA_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/external/lccv/include)
target_link_libraries(camera_module PRIVATE ${OpenCV_LIBS}
                                            ${LIBCAMERA_LIBRARIES} liblccv)
