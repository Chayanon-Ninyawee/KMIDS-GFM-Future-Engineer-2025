add_executable(log_to_video main.cpp)
target_include_directories(log_to_video
                           PRIVATE ${CMAKE_SOURCE_DIR}/src/shared/types)
target_link_libraries(log_to_video PRIVATE log_reader lidar_processor
                                           camera_processor combined_processor)
