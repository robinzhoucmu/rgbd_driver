cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(rgbd)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

set (CMAKE_CXX_STANDARD 14)

if(NOT CMAKE_BUILD_TYPE) 
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${CMAKE_INSTALL_PREFIX}/include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_subdirectory(src)

add_executable(test_real_sense
  test_real_sense.cc
  src/sensor_bridge/rgbd_display_app.cc)
target_link_libraries(test_real_sense sensor_bridge)

