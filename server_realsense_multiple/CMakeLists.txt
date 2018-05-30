# cmake needs this line
cmake_minimum_required(VERSION 2.8)

# Define project name
project(server_realsense_multiple)

# Set installation folder
set (CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "default install path" FORCE )

# Set additional include folders
set (ADDITIONAL_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/include")

include_directories(${ADDITIONAL_INCLUDE})

# Declare the executable target built from your sources
set(SOURCES main.cpp position.pb.cc ${CMAKE_CURRENT_SOURCE_DIR}/proto/position.pb.h)
add_executable(server_realsense_multiple ${SOURCES} )

# Link your application with OpenCV libraries
target_link_libraries(server_realsense_multiple ws2_32)