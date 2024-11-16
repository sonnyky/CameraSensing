cmake_minimum_required(VERSION 2.8)
project (camera_sensing)
 
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
 
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# PCL
set(PCL_DIR "D:/Libraries/pcl64/PCL 1.8.1/cmake")
find_package(PCL 1.8.1 REQUIRED)

# GFLAGS
set(gflags_DIR "D:/Libraries/gflags/install/lib/cmake/gflags")
find_package(gflags REQUIRED)
set(GFLAGS_INCLUDE "D:/Libraries/gflags/install/include")
set(GFLAGS_LIB "D:/Libraries/gflags/install/lib/gflags_nothreads_static.lib")

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
set( OpenCV_DIR "D:/Libraries/x64/opencv/build" )
option( OpenCV_STATIC OFF )
find_package(OpenCV REQUIRED)

# Realsense SDK
set(RealsenseSDK_ROOT "D:/Workspace/librealsense")
set(RealsenseSDK_INCLUDE ${RealsenseSDK_ROOT}/include) #currently hardcoding for Release x64
set(RealsenseSDK_LIB "D:/Libraries/realsense_sdk_x64/Release/realsense2.lib")


# Realsense capture
set(REALSENSE_CAPTURE_DIR "D:/Workspace/CameraSensing/realsense_multiple")
set(REALSENSE_CAPTURE_INCLUDE "D:/Workspace/CameraSensing/realsense_multiple/include")

# PugiXML
set(PUGI_DIR "D:/Libraries/pugixml-1.10/pugixml-1.10/src")


add_subdirectory(calculate_homography)
add_subdirectory(realsense_d415)
add_subdirectory(uplugin_depth_filter)
add_subdirectory(uplugin_cv_util)
add_subdirectory(uplugin_cv_depth)
add_subdirectory(uplugin_face)
add_subdirectory(uplugin_shape)
add_subdirectory(realsense_multiple)
add_subdirectory(merge_realsense_multiple)
add_subdirectory(server_realsense_multiple)
add_subdirectory(plane_detection)
add_subdirectory(position_detector)
add_subdirectory(human_pose_detection)
add_subdirectory(dynamic_projection)
add_subdirectory(reconstruction)
add_subdirectory(reconstruction_markered)
