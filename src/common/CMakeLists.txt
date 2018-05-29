add_library(rgbd_common intrinsics.cc)
target_link_libraries(rgbd_common ${PCL_LIBRARIES}
  ${OpenCV_LIBS})
#
