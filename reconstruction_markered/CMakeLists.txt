# cmake needs this line
cmake_minimum_required(VERSION 2.8)

# Define project name
project(reconstruction_markered)

# Set installation folder
set (CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "default install path" FORCE )

# Set additional include folders
set (ADDITIONAL_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/include")

include_directories(${ADDITIONAL_INCLUDE})
include_directories(${RealsenseSDK_INCLUDE})
include_directories(${PUGI_DIR})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


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
set(SOURCES main.cpp app.cpp ${ADDITIONAL_INCLUDE}/app.h ${ADDITIONAL_INCLUDE}/util.h)
add_executable(reconstruction_markered ${SOURCES} )

# Link your application with OpenCV libraries
target_link_libraries(reconstruction_markered ${OpenCV_LIBS} ${RealsenseSDK_LIB})