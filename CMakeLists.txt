cmake_minimum_required(VERSION 3.26)
project (camera_sensing)
 
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
 
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# GFLAGS
set(gflags_DIR "C:/Users/Sonny/Desktop/Applications/gflags/build")
find_package(gflags REQUIRED)
set(GFLAGS_INCLUDE "C:/Users/Sonny/Desktop/Applications/gflags/build/include")
set(GFLAGS_LIB "C:/Users/Sonny/Desktop/Applications/gflags/build/lib/Release/gflags_nothreads_static.lib")

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
set( OpenCV_DIR "C:/Users/Sonny/Desktop/Applications/opencv_build" )
option( OpenCV_STATIC OFF )
find_package(OpenCV REQUIRED)

add_subdirectory(dynamic_projection)