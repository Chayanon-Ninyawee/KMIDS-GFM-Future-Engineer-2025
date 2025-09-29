add_executable(obstacle_challenge main.cpp)
target_include_directories(obstacle_challenge PRIVATE)
target_link_libraries(
  obstacle_challenge
  PRIVATE lidar_module
          lidar_processor
          pico2_module
          camera_module
          camera_processor
          combined_processor
          pid_controller)
