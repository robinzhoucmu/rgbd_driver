add_library(sensor_bridge 
  rgbd_bridge.cc
  real_sense_d400.cc
  real_sense_common.cc)

target_link_libraries(sensor_bridge
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  rgbd_common
  realsense2)