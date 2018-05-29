#include "sensor_bridge/real_sense_d400.h"
#include "sensor_bridge/real_sense_common.h"
#include "sensor_bridge/scoped_singleton.h"
#include "common/point_cloud_helpers.h"

#include <fstream>
#include <iostream>

#include <boost/make_shared.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <librealsense2/rsutil.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>

namespace rgbd {
namespace sensor_bridge {
namespace {

common::Intrinsics MakeIntrinsics(const rs2_intrinsics& rs_intrin) {
  common::Intrinsics::DistortionModel model;
  switch (rs_intrin.model) {
    case RS2_DISTORTION_NONE:
      model = common::Intrinsics::DistortionModel::NONE;
      break;
    case RS2_DISTORTION_MODIFIED_BROWN_CONRADY:
      model = common::Intrinsics::DistortionModel::MODIFIED_BROWN_CONRADY;
      break;
    case RS2_DISTORTION_INVERSE_BROWN_CONRADY:
      model = common::Intrinsics::DistortionModel::INVERSE_BROWN_CONRADY;
      break;
    case RS2_DISTORTION_FTHETA:
      model = common::Intrinsics::DistortionModel::FTHETA;
      break;
    case RS2_DISTORTION_BROWN_CONRADY:
      model = common::Intrinsics::DistortionModel::BROWN_CONRADY;
      break;
    default:
      throw std::runtime_error("Unknown distortion model.");
  }

  std::array<float, 5> coeffs;
  for (size_t i = 0; i < coeffs.size(); i++) coeffs.at(i) = rs_intrin.coeffs[i];

  return common::Intrinsics(rs_intrin.width, rs_intrin.height, rs_intrin.fx,
                    rs_intrin.fy, rs_intrin.ppx, rs_intrin.ppy, model, coeffs);
}

boost::shared_ptr<cv::Mat> MakeImg(const void* src, int width, int height,
                                   rs2_format pixel_type) {
  int cv_type = -1;
  switch (pixel_type) {
    case RS2_FORMAT_Z16:
    case RS2_FORMAT_DISPARITY16:
    case RS2_FORMAT_Y16:
    case RS2_FORMAT_RAW16:
      cv_type = CV_16UC1;
      break;
    case RS2_FORMAT_Y8:
    case RS2_FORMAT_RAW8:
      cv_type = CV_8UC1;
      break;
    case RS2_FORMAT_RGB8:
    case RS2_FORMAT_BGR8:
      cv_type = CV_8UC3;
      break;
    case RS2_FORMAT_RGBA8:
    case RS2_FORMAT_BGRA8:
      cv_type = CV_8UC4;
      break;
    default:
      throw std::runtime_error("Doesn't support this type");
  }

  boost::shared_ptr<cv::Mat> img =
      boost::make_shared<cv::Mat>(height, width, cv_type);
  memcpy(img->data, src, img->total() * img->elemSize());

  return img;
}

std::shared_ptr<rs2::context> GetRealSense2Context() {
  return drake::GetScopedSingleton<rs2::context>();
}

}  // namespace

RealSenseD400::RealSenseD400(int camera_id)
    : RGBDSensor({ImageType::RGB, ImageType::DEPTH}),
      context_(GetRealSense2Context() ? GetRealSense2Context()
                                      : std::make_shared<rs2::context>()),
      pipeline_(*context_),
      camera_(context_->query_devices()[camera_id]),
      depth_sensor_(camera_.first<rs2::depth_sensor>()),
      camera_name_(camera_.get_info(RS2_CAMERA_INFO_NAME)),
      serial_number_(camera_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
  std::ifstream config_stream;
  if (camera_name_.compare("Intel RealSense D415") == 0) {
    config_stream.open("cfg/d415_high_accuracy2.json");
  } else if (camera_name_.compare("Intel RealSense D435") == 0) {
    config_stream.open("cfg/d435_high_accuracy.json");
  } else {
    throw std::runtime_error(camera_name_ + " is not a D415 or D435");
  }

  if (camera_.is<rs400::advanced_mode>()) {
    auto advanced_mode_dev = camera_.as<rs400::advanced_mode>();
    if (!advanced_mode_dev.is_enabled()) {
      advanced_mode_dev.toggle_advanced_mode(true);
    }

    std::stringstream buffer;
    buffer << config_stream.rdbuf();
    std::string json_config = buffer.str();
    std::cout << "Json config:\n" << json_config << "\n";
    advanced_mode_dev.load_json(json_config);
  } else {
    std::cout << camera_name_ << " did not load config.\n";
  }

  post_process_ = true;

  // Want these streams.
  config_.enable_device(camera_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
  // Use high resolution.
  config_.enable_stream(RS2_STREAM_COLOR, -1, 1280, 720, RS2_FORMAT_RGB8, 30);
  config_.enable_stream(RS2_STREAM_DEPTH, -1, 1280, 720, RS2_FORMAT_ANY, 30);

  auto pipeline_profile = config_.resolve(pipeline_);

  // Getting depth scale.
  depth_scale_ = depth_sensor_.get_option(RS2_OPTION_DEPTH_UNITS);

  // Find camera intrinsics
  std::vector<rs2::stream_profile> profiles = pipeline_profile.get_streams();
  std::map<rs2_stream, rs2_intrinsics> rs_intrinsics;
  for (const auto& profile : profiles) {
    stream_profiles_[profile.stream_type()] = profile;
    rs_intrinsics[profile.stream_type()] =
        profile.as<rs2::video_stream_profile>().get_intrinsics();
  }

  const auto& supported_types = get_supported_image_types();
  for (const auto& type : supported_types) {
    rs2_stream stream = ImageTypeToStreamType(type);
    // Read camera's onboard intrinsics.
    set_intrinsics(type, MakeIntrinsics(rs_intrinsics.at(stream)));

    // Read camera's onboard extrinsics.
    for (const auto& to_type : supported_types) {
      rs2_stream to_stream = ImageTypeToStreamType(to_type);
      const auto rs_extrinsics = stream_profiles_.at(stream).get_extrinsics_to(
          stream_profiles_.at(to_stream));
      set_extrinsics(type, to_type,
                     real_sense::rs_extrinsics_to_eigen(rs_extrinsics));
    }
  }
}

void RealSenseD400::SetMode(const rs2_rs400_visual_preset mode) {
  throw std::runtime_error("Unimplemented");
  // TODO(siyuan): figure out why this doesn't seem to do anything.
  //  depth_sensor_.set_option(RS2_OPTION_VISUAL_PRESET, mode);
  //  auto val = depth_sensor_.get_option(RS2_OPTION_VISUAL_PRESET);
  //  std::cout <<
  //  depth_sensor_.get_option_description(RS2_OPTION_VISUAL_PRESET) << ": "
  //            <<
  //            depth_sensor_.get_option_value_description(RS2_OPTION_VISUAL_PRESET,
  //            val) << "\n";
}

int RealSenseD400::get_number_of_cameras() {
  return GetRealSense2Context()->query_devices().size();
}

void RealSenseD400::DoStart(const std::vector<ImageType>& types) {
  if (run_) {
    return;
  }

  run_ = true;

  std::cout << camera_name_ << " " << serial_number_ << " starting.\n";
  pipeline_.start(config_);

  // Initialize stuff
  for (const ImageType type : types) {
    images_[type] = TimeStampedImage();
  }

  thread_ = std::thread(&RealSenseD400::PollingThread, this);
}

void RealSenseD400::Stop() {
  run_ = false;
  thread_.join();
  pipeline_.stop();
  // Clean up.
  images_.clear();
  cloud_ = TimeStampedCloud();
}

rs2_stream RealSenseD400::ImageTypeToStreamType(const ImageType type) const {
  switch (type) {
    case ImageType::RGB:
      return RS2_STREAM_COLOR;
    case ImageType::DEPTH:
      return RS2_STREAM_DEPTH;
    case ImageType::IR:
      return RS2_STREAM_INFRARED;
    default:
      throw std::logic_error(ImageTypeToString(type) + " is not supported");
  }
}

bool RealSenseD400::is_enabled(const ImageType type) const {
  if (type == ImageType::RGB || type == ImageType::DEPTH) {
    // TODO(siyuan): should querry this from the device.
    return run_;
  }
  return false;
}

void RealSenseD400::PollingThread() {
  std::map<const ImageType, TimeStampedImage> images = images_;

  rs2::frameset frameset;
  std::map<rs2_stream, rs2::frame> frames;

  rs2::temporal_filter low_pass_filter;
  rs2::spatial_filter spatial_filter;
  rs2::disparity_transform depth_to_disparity(true);
  rs2::disparity_transform disparity_to_depth(false);
  low_pass_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.9);
  while (run_) {
    // Block until all frames have arrived.
    frameset = pipeline_.wait_for_frames();

    if (frameset.size() != stream_profiles_.size()) continue;

    for (auto& pair : stream_profiles_) {
      rs2_stream stream = pair.first;
      frames[stream] = frameset.first(stream);
    }

    // Raw color and depth img.
    rs2::frame depth_frame = frames.at(RS2_STREAM_DEPTH);
    if (post_process_) {
      depth_frame = depth_to_disparity.process(depth_frame);
      depth_frame = spatial_filter.process(depth_frame);
      depth_frame = low_pass_filter.process(depth_frame);
      depth_frame = disparity_to_depth.process(depth_frame);
    }

    // Make cv::Mat of the images.
    uint64_t depth_time = -1;
    uint64_t depth_ctr = -1;
    for (auto& pair : images) {
      rs2_stream stream = ImageTypeToStreamType(pair.first);
      const rs2::video_frame frame = frames.at(stream).as<rs2::video_frame>();
      pair.second.timestamp = (uint64_t)frame.get_timestamp();
      pair.second.count = frame.get_frame_number();
      boost::shared_ptr<cv::Mat> img =
          MakeImg(frame.get_data(), frame.get_width(), frame.get_height(),
                  stream_profiles_.at(stream).format());
      // Scale depth image to units of mm.
      // This could cause on overflow if distance is > 65 ish meters.
      if (is_depth_image(pair.first)) {
        real_sense::ScaleImgInPlace(img.get(), depth_scale_ * 1e3);
        depth_time = pair.second.timestamp;
        depth_ctr = pair.second.count;
      }

      pair.second.data = img;
    }

    const cv::Mat& depth_image = *(images.at(ImageType::DEPTH).data);
    const cv::Mat& color_image = *(images.at(ImageType::RGB).data);
    const common::Intrinsics& depth_intrin = get_intrinsics(ImageType::DEPTH);
    const common::Intrinsics& color_intrin = get_intrinsics(ImageType::RGB);
    const Eigen::Isometry3f& depth_to_color =
        get_extrinsics(ImageType::DEPTH, ImageType::RGB);
    const Eigen::Isometry3f& depth_to_desired =
        get_extrinsics(ImageType::DEPTH, get_cloud_camera());

    // Make point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr organized_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

    DoMakePointCloud(get_ignore_depth_not_in_rgb(), color_intrin, depth_intrin,
                     depth_to_color, depth_to_desired, 1e-3,
                     get_clipping_depth_near(), get_clipping_depth_far(),
                     color_image, depth_image, &cloud, &organized_cloud);

    // Lock and do pointer assignment.
    std::unique_lock<std::mutex> lock2(lock_);
    images_ = images;

    cloud_.timestamp = depth_time;
    cloud_.count = depth_ctr;
    cloud_.data = cloud;

    organized_cloud_.timestamp = depth_time;
    organized_cloud_.count = depth_ctr;
    organized_cloud_.data = organized_cloud;
  }
}

boost::shared_ptr<const cv::Mat> RealSenseD400::DoGetLatestImage(
    const ImageType type, uint64_t* timestamp) const {
  std::unique_lock<std::mutex> lock2(lock_);
  auto it = images_.find(type);
  if (it == images_.end()) {
    throw std::runtime_error("ImageType not initialized.");
  }
  const TimeStampedImage& image = it->second;

  if (image.count == 0) {
    *timestamp = 0;
    return nullptr;
  }
  *timestamp = image.timestamp;
  return image.data;
}

pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr
RealSenseD400::DoGetLatestPointCloud(uint64_t* timestamp) const {
  std::unique_lock<std::mutex> lock2(lock_);
  if (cloud_.count == 0) {
    *timestamp = 0;
    return nullptr;
  }
  *timestamp = cloud_.timestamp;
  return cloud_.data;
}

pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr
RealSenseD400::DoGetLatestOrganizedPointCloud(uint64_t* timestamp) const {
  std::unique_lock<std::mutex> lock2(lock_);
  if (organized_cloud_.count == 0) {
    *timestamp = 0;
    return nullptr;
  }
  *timestamp = organized_cloud_.timestamp;
  return organized_cloud_.data;
}
} // namespace sensor_bridege
}  // namespace rgbd
