#pragma once

#include <memory>
#include <vector>

#include "sensor_bridge/rgbd_bridge.h"
namespace rgbd {
namespace sensor_bridge {

/// Run the main loop to display all streams from an RGBD sensor.  For
/// each sensor in @p devices, open one window for each supported
/// image type and display the image stream.  If only one sensor is
/// specified in @p devices, its point cloud data will also be
/// displayed.  The user can left click in the image display to show
/// the pixel coordinate, or right click to save the image to disk.
void RunRgbdDisplay(const std::vector<std::unique_ptr<RGBDSensor>>& devices,
                    bool use_organized_cloud_for_viz = false);

} // namespace sensor_bridge
} // namespace rgbd