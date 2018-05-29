#include "sensor_bridge/rgbd_bridge.h"
namespace rgbd {
namespace sensor_bridge {

std::string ImageTypeToString(const ImageType type) {
  switch (type) {
    case ImageType::RGB:
      return "RGB";
    case ImageType::DEPTH:
      return "DEPTH";
    case ImageType::IR:
      return "INFRARED";
    default:
      throw std::runtime_error("Unknown ImageType");
  }
}

bool is_color_image(const ImageType type) {
  switch (type) {
    case ImageType::RGB:
      return true;
    case ImageType::IR:
    case ImageType::DEPTH:
      return false;
    default:
      throw std::runtime_error("Unknown ImageType");
  }
}

bool is_depth_image(const ImageType type) {
  switch (type) {
    case ImageType::IR:
    case ImageType::RGB:
      return false;
    case ImageType::DEPTH:
      return true;
    default:
      throw std::runtime_error("Unknown ImageType");
  }
}

RGBDSensor::RGBDSensor(const std::vector<ImageType>& supported_types)
    : supported_types_(supported_types) {
  bool supports_depth = false;
  bool supports_color = false;

  for (const auto& from_type : supported_types) {
    for (const auto& to_type : supported_types) {
      set_extrinsics(from_type, to_type, Eigen::Isometry3f::Identity());
    }

    if (is_depth_image(from_type)) supports_depth = true;
    if (is_color_image(from_type)) {
      if (!supports_color) cloud_camera_ = from_type;
      supports_color = true;
    }
  }

  if (!supports_depth) {
    throw std::runtime_error(
        "RGBDSensor has to support at least one depth ImageType.");
  }

  if (!supports_color) {
    throw std::runtime_error(
        "RGBDSensor has to support at least one color ImageType.");
  }
}

void RGBDSensor::Start(const std::vector<ImageType>& types,
                       const ImageType cloud_camera) {
  for (const auto& type : types) {
    if (!supports(type)) {
      throw std::runtime_error("Does not support type: " +
                               ImageTypeToString(type));
    }
  }
  if (!supports(cloud_camera)) {
    throw std::runtime_error("Does not support type: " +
                             ImageTypeToString(cloud_camera));
  }
  cloud_camera_ = cloud_camera;
  DoStart(types);

  std::cout << "Starting with:\n";
  for (const auto& type : types) {
    if (!is_enabled(type)) {
      std::cout << ImageTypeToString(type) << " not enabled.\n";
      continue;
    }

    if (!has_intrinsics(type)) {
      throw std::runtime_error("Missing intrinsics for: " +
                               ImageTypeToString(type));
    }
    std::cout << ImageTypeToString(type) << " enabled with intrinsics: "
              << get_intrinsics(type) << "\n";
  }
}

bool RGBDSensor::supports(const ImageType type) const {
  return std::find(supported_types_.begin(), supported_types_.end(), type) !=
         supported_types_.end();
}

boost::shared_ptr<const cv::Mat> RGBDSensor::GetLatestImage(
    const ImageType type, uint64_t* timestamp) const {
  return DoGetLatestImage(type, timestamp);
}

pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr RGBDSensor::GetLatestPointCloud(
    uint64_t* timestamp) const {
  return DoGetLatestPointCloud(timestamp);
}

pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr
RGBDSensor::GetLatestOrganizedPointCloud(uint64_t* timestamp) const {
  return DoGetLatestOrganizedPointCloud(timestamp);
}

} // namespace sensor_bridge
} // namespace rgbd
