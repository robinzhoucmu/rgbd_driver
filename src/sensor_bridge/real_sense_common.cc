#include "sensor_bridge/real_sense_common.h"

#include <limits>

#include <boost/make_shared.hpp>
#include <pcl/common/transforms.h>

namespace rgbd {
namespace sensor_bridge {
namespace real_sense {
namespace {

template <typename Scalar>
void ScaleImg(cv::Mat* img, float scale) {
  for (int r = 0; r < img->rows; r++) {
    for (int c = 0; c < img->cols; c++) {
      img->at<Scalar>(r, c) *= scale;
    }
  }
}

}  // namespace

void ScaleImgInPlace(cv::Mat* img, float scale) {
  switch (img->type()) {
    case CV_8UC4:
      ScaleImg<cv::Vec4b>(img, scale);
      break;
    case CV_8UC3:
      ScaleImg<cv::Vec3b>(img, scale);
      break;
    case CV_8UC1:
      ScaleImg<uint8_t>(img, scale);
      break;
    case CV_16UC1:
      ScaleImg<uint16_t>(img, scale);
      break;
    default:
      throw std::runtime_error("Doesn't support this type");
  }
}

} // namespace real_sense
} // namespace sensor_bridge
} // namespace rgbd
