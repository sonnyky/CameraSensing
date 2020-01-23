# cmake needs this line
cmake_minimum_required(VERSION 2.8)

# Define project name
project(human_pose_detection)
set(Boost_DEBUG 1)
# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
set( OpenCV_DIR "D:/Libraries/openvino/openvino_2019.3.379/opencv" )
option( OpenCV_STATIC OFF )
find_package(OpenCV REQUIRED)

# OPENVINO
set(INFERENCE_ENGINE_INCLUDE "D:/Libraries/openvino/openvino_2019.3.379/deployment_tools/inference_engine/include")
set(INFERENCE_ENGINE_LIB "D:/Libraries/openvino/openvino_2019.3.379/deployment_tools/inference_engine/lib/intel64/Release/inference_engine.lib")
set(INFERENCE_ENGINE_TBB_LIB "D:/Libraries/openvino/openvino_2019.3.379/deployment_tools/inference_engine/lib/intel64/Release/tbb.lib")
set(INFERENCE_ENGINE_TBBMALLOC_LIB "D:/Libraries/openvino/openvino_2019.3.379/deployment_tools/inference_engine/lib/intel64/Release/tbbmalloc.lib")

# GFLAGS
set(gflags_DIR "D:/Libraries/gflags/install/lib/cmake/gflags")
find_package(gflags REQUIRED)
set(GFLAGS_INCLUDE "D:/Libraries/gflags/install/include")
set(GFLAGS_LIB "D:/Libraries/gflags/install/lib/gflags_nothreads_static.lib")


# PCL
set(PCL_DIR "D:/Libraries/pcl64/PCL 1.8.1/cmake")
find_package(PCL 1.8.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Realsense SDK
set(RealsenseSDK_ROOT "D:/Workspace/librealsense")
set(RealsenseSDK_INCLUDE ${RealsenseSDK_ROOT}/include) #currently hardcoding for Release x64
set(RealsenseSDK_LIB "D:/Libraries/realsense_sdk_x64/Release/realsense2.lib")

# Set installation folder
set (CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "default install path" FORCE )

# Set additional include folders
set (ADDITIONAL_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/include")

include_directories(${ADDITIONAL_INCLUDE})
include_directories(${INFERENCE_ENGINE_INCLUDE})
include_directories(${RealsenseSDK_INCLUDE})
include_directories(${GFLAGS_INCLUDE})

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
set(SOURCES main.cpp pose_detector.cpp human_pose.cpp peak.cpp render_human_pose.cpp ${ADDITIONAL_INCLUDE}/pose_detector.h ${ADDITIONAL_INCLUDE}/human_pose.hpp ${ADDITIONAL_INCLUDE}/peak.hpp ${ADDITIONAL_INCLUDE}/pose_detector_flags.hpp ${ADDITIONAL_INCLUDE}/render_human_pose.hpp)
add_executable(human_pose_detection ${SOURCES} )

# Link your application with OpenCV libraries
target_link_libraries(human_pose_detection ${OpenCV_LIBS} ${RealsenseSDK_LIB} ${PCL_LIBRARIES} ${GFLAGS_LIB} ${INFERENCE_ENGINE_LIB} ${INFERENCE_ENGINE_TBBMALLOC_LIB} ${INFERENCE_ENGINE_TBB_LIB} shlwapi.lib)