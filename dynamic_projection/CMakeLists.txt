cmake_minimum_required(VERSION 3.26)

project(dynamic_projection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "default install path" FORCE)

set(ADDITIONAL_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/include")

# ---- Dependencies ----
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV_DIR (cache): ${OpenCV_DIR}")
message(STATUS "OpenCV_VERSION: ${OpenCV_VERSION}")
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")

# gflags exports un-namespaced targets by default on some installations.
# Request the stable namespaced alias, including the generic gflags::gflags target.
set(GFLAGS_USE_TARGET_NAMESPACE TRUE)
find_package(gflags CONFIG REQUIRED)


# ---- Messages (optional) ----
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

message(STATUS "gflags target selected: gflags::gflags")

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
  calibration_view_selection.cpp
  ${ADDITIONAL_INCLUDE}/dynamic_projection.hpp
  ${ADDITIONAL_INCLUDE}/state.hpp
  ${ADDITIONAL_INCLUDE}/capture.hpp
  ${ADDITIONAL_INCLUDE}/flags.hpp
  ${ADDITIONAL_INCLUDE}/camera_calibration.hpp
  ${ADDITIONAL_INCLUDE}/projector_calibration.hpp
  ${ADDITIONAL_INCLUDE}/calibration.hpp
  ${ADDITIONAL_INCLUDE}/calibration_view_selection.hpp
)

add_executable(dynamic_projection ${SOURCES})

# Target-scoped includes (instead of global include_directories)
target_include_directories(dynamic_projection PRIVATE
  ${ADDITIONAL_INCLUDE}
  ${REALSENSE_CAPTURE_INCLUDE}
  ${OpenCV_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(dynamic_projection PRIVATE
  ${OpenCV_LIBS}
  gflags::gflags
  shlwapi.lib
)
