# NOTE: lidar_controller

# Building rplidar_sdk for lidar_controller
set(RPLIDAR_SDK_PATH ${CMAKE_SOURCE_DIR}/external/rplidar_sdk/sdk)
include_directories(${RPLIDAR_SDK_PATH}/include)
include_directories(${RPLIDAR_SDK_PATH}/src)
file(GLOB_RECURSE RPLIDAR_SDK_SOURCES ${RPLIDAR_SDK_PATH}/src/*.cpp)
list(FILTER RPLIDAR_SDK_SOURCES EXCLUDE REGEX ".*macOS.*")
list(FILTER RPLIDAR_SDK_SOURCES EXCLUDE REGEX ".*win32.*")
add_library(rplidar_sdk STATIC ${RPLIDAR_SDK_SOURCES})

add_library(lidar_controller STATIC lidar_controller.cpp lidar_controller.h)
target_include_directories(
  lidar_controller
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
         ${CMAKE_SOURCE_DIR}/external/rplidar_sdk/sdk/include
         ${CMAKE_SOURCE_DIR}/src/types)
target_link_libraries(lidar_controller PRIVATE rplidar_sdk)
