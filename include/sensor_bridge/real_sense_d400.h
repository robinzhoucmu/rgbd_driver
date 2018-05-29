#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <librealsense2/rs.hpp>

#include "sensor_bridge/rgbd_bridge.h"

namespace rgbd {
namespace sensor_bridge {

// TODO(siyuan): support IR and IR1

/**
 * Only tested to work with D435 and D415 for now.
 *
 * Notes:
 * The depth images and point cloud is set to be post processed by default.
 * Camera settings are saved in json files in /cfg. These config files are
 * hard coded at the moment, can be exposed later if desired. The configs
 * are generated by using Intel's realsense-viewer executable, from which you
 * can experiment with different knobs, and generate a json file.
 *
 * Interference:
 * SR300: yes, some (SR picks up the dots projected by the Ds, but not too bad)
 * D400: no.
 * Mocap: no.
 * Sun: no.
 */
class RealSenseD400 : public RGBDSensor {
 public:
  explicit RealSenseD400(int camera_id);

  ~RealSenseD400() override = default;

  void Stop() override;

  bool is_enabled(const ImageType type) const override;

  static int get_number_of_cameras();

  const std::string& camera_id() const override { return serial_number_; }

 private:
  void DoStart(const std::vector<ImageType>& types) override;

  // TODO(siyuan): figure out how to make this work
  void SetMode(const rs2_rs400_visual_preset mode);

  boost::shared_ptr<const cv::Mat> DoGetLatestImage(
      const ImageType type, uint64_t* timestamp) const override;

  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr DoGetLatestPointCloud(
      uint64_t* timestamp) const override;

  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr DoGetLatestOrganizedPointCloud(
      uint64_t* timestamp) const override;

  rs2_stream ImageTypeToStreamType(const ImageType type) const;

  void PollingThread();

  std::shared_ptr<rs2::context> context_;
  rs2::pipeline pipeline_;
  rs2::device camera_;
  rs2::config config_;
  rs2::depth_sensor depth_sensor_;
  const std::string camera_name_;
  const std::string serial_number_;

  bool post_process_{false};

  std::map<rs2_stream, rs2::stream_profile> stream_profiles_;

  double depth_scale_;

  std::atomic<bool> run_{false};
  mutable std::mutex lock_;
  std::thread thread_;
  std::map<const ImageType, TimeStampedImage> images_;
  TimeStampedCloud cloud_{};
  TimeStampedCloud organized_cloud_{};
};

} // namespace sensor_bridge
} // namespace rgbd