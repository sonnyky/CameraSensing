# cmake needs this line
cmake_minimum_required(VERSION 2.8)

# Define project name
project(position_detector)

set(Boost_DEBUG 1)
# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
set( OpenCV_DIR "D:/Libraries/x64/opencv/build" )
option( OpenCV_STATIC OFF )
find_package(OpenCV REQUIRED)


# PCL
set(PCL_DIR "D:/Libraries/pcl64/PCL 1.8.1/cmake")
find_package(PCL 1.8.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# GFLAGS
set(gflags_DIR "D:/Libraries/gflags/install/lib/cmake/gflags")
find_package(gflags REQUIRED)
set(GFLAGS_INCLUDE "D:/Libraries/gflags/install/include")
set(GFLAGS_LIB "D:/Libraries/gflags/install/lib/gflags_nothreads_static.lib")

# OPENVINO
set(INFERENCE_ENGINE_INCLUDE "D:/Libraries/openvino/openvino_2019.3.379/deployment_tools/inference_engine/include")
set(INFERENCE_ENGINE_LIB "D:/Libraries/openvino/openvino_2019.3.379/deployment_tools/inference_engine/lib/intel64/Release/inference_engine.lib")
set(INFERENCE_ENGINE_TBB_LIB "D:/Libraries/openvino/openvino_2019.3.379/deployment_tools/inference_engine/lib/intel64/Release/tbb.lib")
set(INFERENCE_ENGINE_TBBMALLOC_LIB "D:/Libraries/openvino/openvino_2019.3.379/deployment_tools/inference_engine/lib/intel64/Release/tbbmalloc.lib")


# Realsense SDK
set(RealsenseSDK_ROOT "D:/Workspace/librealsense")
set(RealsenseSDK_INCLUDE ${RealsenseSDK_ROOT}/include) #currently hardcoding for Release x64
set(RealsenseSDK_LIB "D:/Libraries/realsense_sdk_x64/Release/realsense2.lib")

# Realsense capture
set(REALSENSE_CAPTURE_DIR "D:/Workspace/CameraSensing/realsense_multiple")
set(REALSENSE_CAPTURE_INCLUDE "D:/Workspace/CameraSensing/realsense_multiple/include")

# Set installation folder
set (CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "default install path" FORCE )

# Set additional include folders
set (ADDITIONAL_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/include")

include_directories(${ADDITIONAL_INCLUDE})
include_directories(${RealsenseSDK_INCLUDE})
include_directories(${INFERENCE_ENGINE_INCLUDE})
include_directories(${GFLAGS_INCLUDE})
include_directories(${REALSENSE_CAPTURE_INCLUDE})

# Set directories of plane detector library
set (PLANE_DETECTOR "D:/Workspace/CameraSensing/plane_detection")
set (PLANE_DETECTOR_INCLUDE "D:/Workspace/CameraSensing/plane_detection/include")

# Set directories of the human pose detection project
set (HUMAN_POSE_DETECTOR "D:/Workspace/CameraSensing/human_pose_detection")
set (HUMAN_POSE_DETECTOR_INCLUDE "D:/Workspace/CameraSensing/human_pose_detection/include")

include_directories(${PLANE_DETECTOR_INCLUDE})
include_directories(${HUMAN_POSE_DETECTOR_INCLUDE})
include_directories(${ADDITIONAL_INCLUDE})

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

if(CMAKE_VERSION VERSION_LESS "2.8.11")
  # Add OpenCV headers location to your include paths
  include_directories(${OpenCV_INCLUDE_DIRS} ${RealsenseSDK_LIB})
endif()

# Declare the executable target built from your sources
set(SOURCES 
	main.cpp
	position_detector.cpp
	${HUMAN_POSE_DETECTOR}/pose_detector.cpp
	${HUMAN_POSE_DETECTOR}/human_pose.cpp
	${HUMAN_POSE_DETECTOR}/peak.cpp
	${HUMAN_POSE_DETECTOR}/render_human_pose.cpp
	${PLANE_DETECTOR}/include/plane_detection.h 
	${PLANE_DETECTOR}/plane_detection.cpp
	${REALSENSE_CAPTURE_DIR}/app.cpp
	${HUMAN_POSE_DETECTOR_INCLUDE}/pose_detector.hpp
	${HUMAN_POSE_DETECTOR_INCLUDE}/pose_detector_flags.hpp
	${HUMAN_POSE_DETECTOR_INCLUDE}/human_pose.hpp
	${HUMAN_POSE_DETECTOR_INCLUDE}/peak.hpp
	${HUMAN_POSE_DETECTOR_INCLUDE}/render_human_pose.hpp
	${ADDITIONAL_INCLUDE}/position_detector.hpp
	${REALSENSE_CAPTURE_INCLUDE}/app.h
	)

add_executable(position_detector ${SOURCES} )

# Link your application with OpenCV libraries
target_link_libraries(position_detector ${OpenCV_LIBS} ${RealsenseSDK_LIB} ${PCL_LIBRARIES} ${GFLAGS_LIB} ${INFERENCE_ENGINE_LIB} ${INFERENCE_ENGINE_TBBMALLOC_LIB} ${INFERENCE_ENGINE_TBB_LIB} shlwapi.lib)