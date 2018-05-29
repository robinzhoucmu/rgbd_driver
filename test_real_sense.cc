#include "sensor_bridge/real_sense_d400.h"
#include "sensor_bridge/rgbd_display_app.h"

int main(int argc, char** argv) {
  std::vector<std::unique_ptr<rgbd::sensor_bridge::RGBDSensor>> devices;

  const int kNumD400 = rgbd::sensor_bridge::RealSenseD400::get_number_of_cameras();
  std::cout << "TOTAL NUM D400 CAM: " << kNumD400 << "\n";

  for (int i = 0; i < kNumD400; i++) {
    auto real_sense = std::make_unique<rgbd::sensor_bridge::RealSenseD400>(i);
    devices.push_back(std::move(real_sense));
  }

  const bool kUseOrganizedCloud = false;
  rgbd::sensor_bridge::RunRgbdDisplay(devices, kUseOrganizedCloud);
  return 0;
}
