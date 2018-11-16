cmake_minimum_required(VERSION 2.8)

project (uplugin_face)
set(CMAKE_BUILD_TYPE Release)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/uplugin/cv_depth)

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
set(RealsenseSDK_LIB_DIR "D:/Libraries/realsense_sdk_x64/Release")

# Set installation folder
set (CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "default install path" FORCE )

# Set additional include folders
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${RealsenseSDK_INCLUDE})

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

link_directories(${RealsenseSDK_LIB_DIR})

# add_library sets the project to build dll libraries. For executables use add_executable instead
add_library(uplugin_face SHARED main.cpp recognition.cpp detection.cpp ${CMAKE_CURRENT_SOURCE_DIR}/include/recognition.h ${CMAKE_CURRENT_SOURCE_DIR}/include/detection.h ${CMAKE_CURRENT_SOURCE_DIR}/include/common.hpp)

target_link_libraries(uplugin_face ${OpenCV_LIBS}  ${RealsenseSDK_LIB})