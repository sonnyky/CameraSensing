# cmake needs this line
cmake_minimum_required(VERSION 2.8)

# Define project name
project(server_realsense_multiple_dleaf)

# Set installation folder
set (CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "default install path" FORCE )

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
set( OpenCV_DIR "D:/Libraries/x64/opencv/build" )
option( OpenCV_STATIC OFF )
find_package(OpenCV REQUIRED)

# Set additional include folders
set (ADDITIONAL_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/include")

# OSCPACK
set(OSCPACK_INCLUDE_DIR      "D:/Libraries/oscpack_1_1_0")

include_directories(${ADDITIONAL_INCLUDE} ${OSCPACK_INCLUDE_DIR})

# Declare the executable target built from your sources
set(SOURCES main.cpp app.cpp jsoncpp.cpp position.pb.cc ${CMAKE_CURRENT_SOURCE_DIR}/include/server.h ${OSCPACK_INCLUDE_DIR}/osc/OscTypes.cpp ${OSCPACK_INCLUDE_DIR}/osc/OscReceivedElements.cpp ${OSCPACK_INCLUDE_DIR}/osc/OscPrintReceivedElements.cpp ${OSCPACK_INCLUDE_DIR}/osc/OscOutboundPacketStream.cpp ${OSCPACK_INCLUDE_DIR}/ip/IpEndpointName.cpp ${OSCPACK_INCLUDE_DIR}/ip/win32/NetworkingUtils.cpp ${OSCPACK_INCLUDE_DIR}/ip/win32/UdpSocket.cpp ${CMAKE_CURRENT_SOURCE_DIR}/proto/position.pb.h ${ADDITIONAL_INCLUDE}/json/json.h ${ADDITIONAL_INCLUDE}/json/json-forwards.h ${ADDITIONAL_INCLUDE}/json/config.h)
add_executable(server_realsense_multiple_dleaf ${SOURCES} )

# Link your application with OpenCV libraries
target_link_libraries(server_realsense_multiple_dleaf ${OpenCV_LIBS} ws2_32 winmm)