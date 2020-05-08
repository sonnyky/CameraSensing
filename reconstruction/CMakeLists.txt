# cmake needs this line
cmake_minimum_required(VERSION 2.8)

# Define project name
project(reconstruction)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
set( OpenCV_DIR "D:/Libraries/x64/opencv/build" )
option( OpenCV_STATIC OFF )
find_package(OpenCV REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# GFLAGS
set(gflags_DIR "D:/Libraries/gflags/install/lib/cmake/gflags")
find_package(gflags REQUIRED)
set(GFLAGS_INCLUDE "D:/Libraries/gflags/install/include")
set(GFLAGS_LIB "D:/Libraries/gflags/install/lib/gflags_nothreads_static.lib")


# Realsense SDK
set(RealsenseSDK_ROOT "D:/Workspace/librealsense")
set(RealsenseSDK_INCLUDE ${RealsenseSDK_ROOT}/include) #currently hardcoding for Release x64
set(RealsenseSDK_LIB "D:/Libraries/realsense_sdk_x64/Release/realsense2.lib")

# Set installation folder
set (CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "default install path" FORCE )

# Set additional include folders
set (ADDITIONAL_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/include")

include_directories(${ADDITIONAL_INCLUDE})
include_directories(${RealsenseSDK_INCLUDE})
include_directories(${GFLAGS_INCLUDE})
include_directories(${PUGI_DIR})


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
set(SOURCES main.cpp app.cpp pcl_to_mesh.cpp ${PUGI_DIR}/pugixml.cpp ${ADDITIONAL_INCLUDE}/app.h ${ADDITIONAL_INCLUDE}/util.h ${ADDITIONAL_INCLUDE}/pcl_to_mesh.h)
add_executable(reconstruction ${SOURCES} )

# Link your application with OpenCV libraries
target_link_libraries(reconstruction ${OpenCV_LIBS} ${RealsenseSDK_LIB} ${PCL_LIBRARIES} ${GFLAGS_LIB})