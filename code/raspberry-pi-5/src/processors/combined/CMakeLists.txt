# NOTE: combined_processor

add_library(combined_processor STATIC combined_processor.cpp
                                      combined_processor.h)
target_include_directories(
  combined_processor
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS}
         ${CMAKE_SOURCE_DIR}/src/types ${CMAKE_SOURCE_DIR}/src/shared/types)
target_link_libraries(
  combined_processor
  PRIVATE ${OpenCV_LIBS}
  PUBLIC ring_buffer camera_processor lidar_processor)
