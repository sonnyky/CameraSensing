cmake_minimum_required(VERSION 3.26)
project (camera_sensing)
 
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
 
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# Find OpenCV. Supply OpenCV_DIR or CMAKE_PREFIX_PATH when it is not installed
# in a standard CMake search location.
option( OpenCV_STATIC OFF )
find_package(OpenCV REQUIRED)

add_subdirectory(dynamic_projection)
