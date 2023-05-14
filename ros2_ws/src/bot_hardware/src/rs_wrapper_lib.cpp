#include "rs_wrapper_lib.hpp"
#include <iostream>
#include <string.h>

using namespace donkeyjet::hardware;

void RealsenseSensor::stop_stream() {
  rs_pipeline_.stop();
}

void RealsenseSensor::start_stream() {
  if (callback) {
    rs_pipeline_.start(rs_config_, callback);
  } else {
    auto c = std::bind(&RealsenseSensor::default_callback, this,
                       std::placeholders::_1);
    rs_pipeline_.start(rs_config_, c);
  }
}

T265::T265(const rs2::device &rs_device) {
  if (strcmp(rs_device.get_info(RS2_CAMERA_INFO_NAME),
             "Intel RealSense T265") != 0) {
    throw std::runtime_error("Device with a wrong name");
  }
  rs_config_.enable_stream(RS2_STREAM_ACCEL);
  rs_config_.enable_stream(RS2_STREAM_GYRO);
  rs_config_.enable_stream(RS2_STREAM_POSE);
}

void T265::default_callback(const rs2::frame &frame) {
  if (frame.is<rs2::motion_frame>()) {
    auto motion = frame.as<rs2::motion_frame>();
    const auto &stream_type = motion.get_profile().stream_type();
    std::cout << "gyro: " << (stream_type == RS2_STREAM_GYRO)
              << ", accel: " << (stream_type == RS2_STREAM_ACCEL) << std::endl;
  } else if (frame.is<rs2::pose_frame>()) {
    std::cout << "pose frame" << std::endl;
  }
}

RealsenseSensorPtr T265::create(const rs2::device &rs_device) {
  RealsenseSensor *ptr{new T265{rs_device}};
  return std::shared_ptr<RealsenseSensor>(ptr);
}

D435::D435(const rs2::device &rs_device) {
  if (strcmp(rs_device.get_info(RS2_CAMERA_INFO_NAME),
             "Intel RealSense D435") != 0) {
    throw std::runtime_error("Device with a wrong name");
  }
  rs_config_.enable_stream(RS2_STREAM_DEPTH); // , 256, 144, RS2_FORMAT_Z16, 90
}

void D435::default_callback(const rs2::frame &frame) {
  if (frame.is<rs2::frameset>()) {
    std::cout << "Depth data" << std::endl;
  }
}

RealsenseSensorPtr D435::create(const rs2::device &rs_device) {
  RealsenseSensor *ptr{new D435{rs_device}};
  return std::shared_ptr<RealsenseSensor>(ptr);
}
