#pragma once

#include <atomic>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common/intrinsics.h"

namespace rgbd {
namespace sensor_bridge {

enum class ImageType {
  RGB = 0,
  DEPTH,
  IR,
};

std::string ImageTypeToString(const ImageType type);
bool is_color_image(const ImageType type);
bool is_depth_image(const ImageType type);

class RGBDSensor {
 public:
  virtual ~RGBDSensor() {}

  // TODO(sam.creasey) Add code which allows the caller to check which
  // image types were successfully enabled after calling Start().  (I
  // have this partially complete on a branch).

  /// Starts streaming images from the camera.
  ///
  /// @param types which ImageTypes to enable on the camera.  Not all
  /// devices support all image types.
  ///
  /// @param cloud_camera specifies the origin point for resulting point
  /// clouds by the sensor origin for that image type.  Related:
  /// set_extrinsics().
  void Start(const std::vector<ImageType>& types, const ImageType cloud_camera);

  virtual void Stop() = 0;

  /**
   * For rgb image, the channels are in RGB order.
   * For depth image, each element is 16bits, in units of mm.
   */
  boost::shared_ptr<const cv::Mat> GetLatestImage(const ImageType type,
                                                  uint64_t* timestamp) const;

  /**
   * Returns a unorganized point cloud. XYZ units are in m.
   */
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr GetLatestPointCloud(
      uint64_t* timestamp) const;

  /**
   * Returns a organized point cloud with the same resolution as the depth
   * camera. For pixel that doesn't have valid depth values, the corresponding
   * point will have XYZ values as NaN. The XYZ units are in m, NOT in pixel
   * coordinates.
   */
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr GetLatestOrganizedPointCloud(
      uint64_t* timestamp) const;

  const std::vector<ImageType>& get_supported_image_types() const {
    return supported_types_;
  }

  /**
   * Returns true if @p type is supported.
   */
  bool supports(const ImageType type) const;

  /**
   * Projects @p xyz to @p type's image plane.
   * @throws std::runtime_error if @p type does not have associated
   * intrinsics.
   */
  Eigen::Vector2f Project(const ImageType type,
                          const Eigen::Vector3f& xyz) const {
    return get_intrinsics(type).Project(xyz);
  }

  /**
   * Projects @p uv from @p type's image plane to 3D.
   * @param uv in pixel.
   * @param depth in meters.
   * @throws std::runtime_error if @p type does not have associated
   * intrinsics.
   */
  Eigen::Vector3f Backproject(const ImageType type, const Eigen::Vector2i& uv,
                              float depth) const {
    return get_intrinsics(type).BackProject(uv.cast<float>(), depth);
  }

  /**
   * Returns true if @p type has been started by by Start().
   */
  virtual bool is_enabled(const ImageType type) const = 0;

  /**
   * Returns true if there are intrinsics associated with @p type.
   */
  bool has_intrinsics(ImageType type) const {
    return intrinsics_.count(type) > 0;
  }

  /**
   * Returns the intrinsics associated with @p type.
   * @throws std::runtime_error if there is none.
   */
  const common::Intrinsics& get_intrinsics(ImageType type) const {
    return intrinsics_.at(type);
  }

  void set_intrinsics(ImageType type, const common::Intrinsics& intrinsics) {
    intrinsics_[type] = intrinsics;
  }

  bool has_extrinsics(ImageType from, ImageType to) const {
    return extrinsics_.count(std::pair<ImageType, ImageType>(from, to)) > 0;
  }

  /**
   * Returns X_to_from.
   * @throws std::runtime_error if there is no transformation between @p from
   * and @p to.
   */
  const Eigen::Isometry3f& get_extrinsics(ImageType from, ImageType to) const {
    return extrinsics_.at(std::pair<ImageType, ImageType>(from, to));
  }

  /**
   * Sets the extrinsics of X_to_from to @p extrinsics, and X_from_to_ to
   * the inverse of @p extrinsics.
   */
  void set_extrinsics(ImageType from, ImageType to,
                      const Eigen::Isometry3f& extrinsics) {
    extrinsics_[std::pair<ImageType, ImageType>(from, to)] = extrinsics;
    extrinsics_[std::pair<ImageType, ImageType>(to, from)] =
        extrinsics.inverse();
  }

  /**
   * Returns the ImageType where the cloud is based at.
   */
  const ImageType& get_cloud_camera() const { return cloud_camera_; }

  /**
   * Sets a clipping plane in the depth image's frame. All points with a
   * distance further than @p z will be discarded in the generated point cloud.
   * @p z is in meters.
   */
  void set_clipping_depth_far(float z) { clipping_depth_far_ = z; }

  /**
   * Sets a clipping plane in the depth image's frame. All points with a
   * distance closer than @p z will be discarded in the generated point cloud.
   * @p z is in meters.
   */
  void set_clipping_depth_near(float z) { clipping_depth_near_ = z; }

  /**
   * When set to true, points that are not visible by the color camera will
   * be discarded in the generated point cloud.
   */
  void set_ignore_depth_not_in_rgb(bool flag) {
    ignore_depth_not_in_rgb_ = flag;
  }

  float get_clipping_depth_far() const { return clipping_depth_far_; }
  float get_clipping_depth_near() const { return clipping_depth_near_; }
  bool get_ignore_depth_not_in_rgb() const { return ignore_depth_not_in_rgb_; }

  /// Pause all streams from the camera.  This also turns off the
  /// emitter.  Not supported by all cameras.
  virtual void Pause() {}

  /// Resume streaming from the camera.  Not supported by all
  /// cameras.
  virtual void Resume() {}

  /// @return a unique string identifying the camera (typically a
  /// serial number).
  virtual const std::string& camera_id() const = 0;

 protected:
  template <typename DataType>
  struct TimeStampedData {
    DataType data{};
    uint64_t timestamp{0};
    uint64_t count{0};
  };

  /**
   * @p supported_types has to contain at least one depth ImageType and one
   * color ImageType.
   * Defaults extrinsics between all supported ImageType to identity.
   * Defaults cloud_camera_ to RGB if it is available, otherwise to DEPTH.
   * Derived class should override extrinsics_ when the information is
   * available. cloud_camera_ will be set again during in Start().
   */
  // TODO(siyuan): maybe relax at least one color ImageType constraint to
  // IR or grays cale.
  explicit RGBDSensor(const std::vector<ImageType>& supported_types);

  typedef TimeStampedData<boost::shared_ptr<const cv::Mat>> TimeStampedImage;
  typedef TimeStampedData<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr>
      TimeStampedCloud;

  virtual boost::shared_ptr<const cv::Mat> DoGetLatestImage(
      const ImageType type, uint64_t* timestamp) const = 0;
  virtual pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr DoGetLatestPointCloud(
      uint64_t* timestamp) const = 0;
  virtual pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr
  DoGetLatestOrganizedPointCloud(uint64_t* timestamp) const = 0;

  virtual void DoStart(const std::vector<ImageType>& types) = 0;

 private:
  const std::vector<ImageType> supported_types_;
  std::atomic<float> clipping_depth_near_{0};
  std::atomic<float> clipping_depth_far_{
      std::numeric_limits<float>::infinity()};
  std::atomic<bool> ignore_depth_not_in_rgb_{false};

  // TODO(siyuan) should probably lock these for set / get.
  std::map<ImageType, common::Intrinsics> intrinsics_;
  std::map<std::pair<ImageType, ImageType>, Eigen::Isometry3f> extrinsics_;
  ImageType cloud_camera_;
};

} // namespace realsense_bridge
} // namespace rgbd