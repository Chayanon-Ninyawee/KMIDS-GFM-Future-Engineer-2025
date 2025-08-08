add_executable(test_temp main.cpp)
target_include_directories(test_temp PRIVATE)
target_link_libraries(test_temp PRIVATE lidar_module)
