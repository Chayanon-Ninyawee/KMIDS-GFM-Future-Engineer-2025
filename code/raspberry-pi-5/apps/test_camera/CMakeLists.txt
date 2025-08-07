add_executable(test_camera main.cpp)
target_include_directories(test_camera PRIVATE ${CMAKE_SOURCE_DIR}/src/modules)
target_link_libraries(test_camera PRIVATE camera_module)
