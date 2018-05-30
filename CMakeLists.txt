cmake_minimum_required(VERSION 2.8)
project (camera_vision)
 
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
 
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

add_subdirectory(calculate_homography)
add_subdirectory(realsense_d415)
add_subdirectory(uplugin_depth_filter)
add_subdirectory(realsense_multiple)
add_subdirectory(merge_realsense_multiple)
add_subdirectory(server_realsense_multiple)