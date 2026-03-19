cmake_minimum_required(VERSION 3.26)

project(dynamic_projection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "default install path" FORCE)

set(ADDITIONAL_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/include")
# ---- gflags hint (temporary, for CMake GUI stability) ----
set(gflags_DIR "C:/Users/Sonny/Desktop/Applications/gflags-install/install/lib/cmake/gflags"
    CACHE PATH "Path to gflags-config.cmake directory")
message(STATUS "gflags_DIR = '${gflags_DIR}'")

# ---- Dependencies ----
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV_DIR (cache): ${OpenCV_DIR}")
message(STATUS "OpenCV_VERSION: ${OpenCV_VERSION}")
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")

# gflags (installed)
# You can pass -Dgflags_DIR=".../install/lib/cmake/gflags" when configuring
find_package(gflags CONFIG REQUIRED)
# If the package config didn't import targets (happens when config doesn't include targets),
# include the targets file explicitly.
if(NOT TARGET gflags::gflags_static)
  include("${gflags_DIR}/gflags-targets.cmake")
endif()

# Now select the correct exported target
if(TARGET gflags::gflags_static)
  set(GFLAGS_TARGET gflags::gflags_static)
elseif(TARGET gflags::gflags_nothreads_static)
  set(GFLAGS_TARGET gflags::gflags_nothreads_static)
else()
  message(FATAL_ERROR
    "gflags was found at '${gflags_DIR}' but targets are still missing. "
    "Expected gflags::gflags_static or gflags::gflags_nothreads_static."
  )
endif()


# ---- Messages (optional) ----
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

message(STATUS "gflags target selected: ${GFLAGS_TARGET}")

# ---- Sources ----
set(SOURCES
  main.cpp
  capture.cpp
  flags.cpp
  dynamic_projection.cpp
  state.cpp
  camera_calibration.cpp
  projector_calibration.cpp
  calibration.cpp
  ${ADDITIONAL_INCLUDE}/dynamic_projection.hpp
  ${ADDITIONAL_INCLUDE}/state.hpp
  ${ADDITIONAL_INCLUDE}/capture.hpp
  ${ADDITIONAL_INCLUDE}/flags.hpp
  ${ADDITIONAL_INCLUDE}/camera_calibration.hpp
  ${ADDITIONAL_INCLUDE}/projector_calibration.hpp
  ${ADDITIONAL_INCLUDE}/calibration.hpp
)

add_executable(dynamic_projection ${SOURCES})

# Target-scoped includes (instead of global include_directories)
target_include_directories(dynamic_projection PRIVATE
  ${ADDITIONAL_INCLUDE}
  ${REALSENSE_CAPTURE_INCLUDE}
  ${OpenCV_INCLUDE_DIRS}
)

if(NOT TARGET gflags::gflags_static)
  message(FATAL_ERROR "gflags::gflags_static is not defined at link time. gflags_DIR='${gflags_DIR}'")
endif()

# Link libraries
target_link_libraries(dynamic_projection PRIVATE
  ${OpenCV_LIBS}
  ${GFLAGS_TARGET}
  shlwapi.lib
)
