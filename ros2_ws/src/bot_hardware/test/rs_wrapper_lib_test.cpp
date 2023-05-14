#include "rs_wrapper_lib.hpp"
#include <chrono>
#include <gtest/gtest.h>
#include <thread>

using namespace donkeyjet::hardware;

TEST(realsense_lib, test_sensors) {
  rs2::context ctx;
  std::vector<RealsenseSensorPtr> sensors;
  for (auto dev : ctx.query_devices()) {
    std::cout << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
    if (strcmp(dev.get_info(RS2_CAMERA_INFO_NAME), "Intel RealSense D435") ==
        0) {
      auto sensor = D435::create(dev);
      sensors.push_back(sensor);
    } else if (strcmp(dev.get_info(RS2_CAMERA_INFO_NAME),
                      "Intel RealSense T265") == 0) {
      auto sensor = T265::create(dev);
      sensor->callback = [](const rs2::frame &frame) {
        if (frame.is<rs2::motion_frame>()) {
          std::cout << "IMU data in new callback" << std::endl;
        } else if (frame.is<rs2::pose_frame>())
        {
          std::cout << "Pose data in new callback" << std::endl;
        }
      };
      sensors.push_back(sensor);
    }
  }

  for (auto &sensor : sensors) {
    sensor->start_stream();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  for (auto &sensor : sensors) {
    sensor->stop_stream();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}