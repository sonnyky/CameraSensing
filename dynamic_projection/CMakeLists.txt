# cmake needs this line
cmake_minimum_required(VERSION 2.8)

# Define project name
project(dynamic_projection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set installation folder
set (CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "default install path" FORCE )

# Set additional include folders
set (ADDITIONAL_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/include")

include_directories(${RealsenseSDK_INCLUDE})
include_directories(${GFLAGS_INCLUDE})
include_directories(${REALSENSE_CAPTURE_INCLUDE})

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
	utilities.cpp
	dynamic_projection.cpp
	state.cpp
	camera_calibration.cpp
	projector_calibration.cpp
	calibration.cpp
	${REALSENSE_CAPTURE_DIR}/app.cpp
	${ADDITIONAL_INCLUDE}/dynamic_projection.hpp
	${ADDITIONAL_INCLUDE}/state.hpp
	${ADDITIONAL_INCLUDE}/utilities.hpp
	${ADDITIONAL_INCLUDE}/flags.hpp
	${ADDITIONAL_INCLUDE}/camera_calibration.hpp
	${ADDITIONAL_INCLUDE}/projector_calibration.hpp
	${ADDITIONAL_INCLUDE}/calibration.hpp
	${REALSENSE_CAPTURE_INCLUDE}/app.h
	)

add_executable(dynamic_projection ${SOURCES} )

# Link your application with OpenCV libraries
target_link_libraries(dynamic_projection ${OpenCV_LIBS} ${RealsenseSDK_LIB} ${GFLAGS_LIB} shlwapi.lib)