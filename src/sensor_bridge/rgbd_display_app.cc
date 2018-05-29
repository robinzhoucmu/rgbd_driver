#include "sensor_bridge/rgbd_display_app.h"

#include <string>
#include <thread>

#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/pcl_visualizer.h>

namespace rgbd {
namespace sensor_bridge {
namespace {

struct UserClickedPt {
  int x{-1};
  int y{-1};
  bool save_pic{false};
  std::string window_name;
};

cv::Mat proc_depth_image(const sensor_bridge::RGBDSensor &camera,
                         const sensor_bridge::ImageType type,
                         const cv::Mat &raw_depth, const UserClickedPt &input) {
  cv::Mat ret;
  ret = raw_depth;

  raw_depth.convertTo(ret, CV_8UC1, 255. / 1000., 0);
  cv::cvtColor(ret, ret, CV_GRAY2BGR);

  if (input.x >= 0 && input.x < ret.cols && input.y >= 0 &&
      input.y < ret.rows) {
    cv::circle(ret, cv::Point(input.x, input.y), 5, cv::Scalar(0, 255, 0));
    float depth = raw_depth.at<uint16_t>(input.y, input.x) / 1000.;
    Eigen::Vector3f xyz =
        camera.Backproject(type, Eigen::Vector2i(input.x, input.y), depth);

    const std::string text = "xyz: " + std::to_string(xyz(0)) + "," +
                             std::to_string(xyz(1)) + "," +
                             std::to_string(xyz(2));
    cv::putText(ret, text, cv::Point(0, 50), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                cv::Scalar(0, 255, 0));
  }

  return ret;
}

cv::Mat proc_ir_image(const cv::Mat &raw_ir, const UserClickedPt &input) {
  cv::Mat ret;
  cv::cvtColor(raw_ir, ret, CV_GRAY2BGR);
  return ret;
}

cv::Mat proc_rgb_image(const cv::Mat &raw_rgb, const UserClickedPt &input) {
  cv::Mat ret;
  cv::cvtColor(raw_rgb, ret, CV_RGB2BGR);
  return ret;
}

void image_loop(const sensor_bridge::RGBDSensor *driver, int camera_id,
                const std::vector<sensor_bridge::ImageType> channels,
                std::vector<UserClickedPt> *clicked_xy) {
  uint64_t timestamp;

  cv::Mat tmp;

  int ctr = 0;
  while (true) {
    for (size_t i = 0; i < channels.size(); i++) {
      const auto type = channels[i];
      auto img = driver->GetLatestImage(type, &timestamp);
      if (img) {
        if (is_color_image(type)) {
          tmp = proc_rgb_image(*img, clicked_xy->at(i));
        } else if (is_depth_image(type)) {
          tmp = proc_depth_image(*driver, type, *img, clicked_xy->at(i));
        } else if (type == sensor_bridge::ImageType::IR) {
          tmp = proc_ir_image(*img, clicked_xy->at(i));
        }
        if (clicked_xy->at(i).save_pic) {
          if (is_color_image(type)) {
              cv::imwrite(ImageTypeToString(type) + std::to_string(camera_id) +
                          std::to_string(ctr++) + ".jpg", tmp);
          } else {
            cv::imwrite(ImageTypeToString(type) + std::to_string(camera_id) +
                        std::to_string(ctr++) + ".png", *img);
          }
          clicked_xy->at(i).save_pic = false;
        }

        cv::imshow(ImageTypeToString(type) + std::to_string(camera_id), tmp);
      }
    }

    cv::waitKey(5);
  }
}

void cloud_loop(const sensor_bridge::RGBDSensor *driver,
                bool use_organized_cloud) {
  uint64_t timestamp;

  pcl::visualization::PCLVisualizer viewer("Point Cloud Visualization");
  viewer.addCoordinateSystem(0.2, Eigen::Affine3f::Identity());

  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud;
  while (true) {
    if (use_organized_cloud)
      cloud = driver->GetLatestOrganizedPointCloud(&timestamp);
    else
      cloud = driver->GetLatestPointCloud(&timestamp);

    if (cloud) {
      viewer.addPointCloud(cloud, "cloud");
      viewer.spinOnce();
      viewer.removePointCloud("cloud");
    }
  }
}

void mouse_click(int event, int x, int y, int flags, void *userdata) {
  UserClickedPt* data = reinterpret_cast<UserClickedPt*>(userdata);
  if (event == cv::EVENT_LBUTTONDOWN) {
    data->x = x;
    data->y = y;
    std::cout << data->window_name << ": " << x << ", " << y << "\n";
  } else if (event == cv::EVENT_RBUTTONDOWN) {
    data->save_pic = true;
  }
}

}  // namespace

void RunRgbdDisplay(const std::vector<std::unique_ptr<RGBDSensor>>& devices,
                    bool use_organized_cloud_for_viz) {
  std::vector<std::string> names;

  for (size_t i = 0; i < devices.size(); i++) {
    const std::vector<ImageType>& channels =
        devices[i]->get_supported_image_types();

    devices[i]->Start(channels, ImageType::RGB);
    std::cout << "Started camera: " << devices[i]->camera_id() << "\n";

    for (const auto type : channels) {
      if (devices[i]->is_enabled(type)) {
        names.push_back(ImageTypeToString(type) + std::to_string(i));
      }
    }
  }
  for (const auto &name : names) {
    cv::namedWindow(name);
  }

  std::vector<std::thread> img_threads;
  std::vector<std::thread> cloud_threads;

  std::vector<std::vector<UserClickedPt>> inputs(devices.size());

  for (size_t i = 0; i < devices.size(); i++) {
    const std::vector<ImageType>& all_channels =
        devices[i]->get_supported_image_types();
    // Only look at enabled channels.
    std::vector<ImageType> channels;
    std::copy_if(all_channels.begin(), all_channels.end(),
                 std::back_inserter(channels), [&](const auto& image_type) {
                   return devices[i]->is_enabled(image_type);
                 });

    inputs[i] = std::vector<UserClickedPt>(channels.size());
    for (size_t c = 0; c < channels.size(); c++) {
      inputs[i][c].window_name =
          ImageTypeToString(channels[c]) + std::to_string(i);
    }

    img_threads.emplace_back(
        std::thread(image_loop, devices[i].get(), i, channels, (&inputs[i])));
    for (size_t c = 0; c < channels.size(); c++) {
      cv::setMouseCallback(ImageTypeToString(channels[c]) + std::to_string(i),
                           mouse_click, &(inputs[i][c]));
    }
    // Since i can only visualize 1 cloud with pcl's viz.
    if (devices.size() == 1)
      cloud_threads.emplace_back(
          std::thread(cloud_loop, devices[i].get(),
                      use_organized_cloud_for_viz));
  }

  for (auto &thread : img_threads)
    thread.join();

  for (auto &thread : cloud_threads)
    thread.join();
}

} // namespace sensor_bridge
} // namespace rgbd
