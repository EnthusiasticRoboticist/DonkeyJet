#include <gtest/gtest.h>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <iostream>
#include <map>
#include <chrono>
#include <mutex>
#include <thread>

// TEST(realsense_test, imu_test_wait_for_frame_D435i)
// {
//   rs2::pipeline pipe;
//   rs2::config cfg;
//   cfg.enable_stream(RS2_STREAM_ACCEL);
//   cfg.enable_stream(RS2_STREAM_GYRO);
//   pipe.start(cfg);

//   while (true) // Application still alive?
//   {
//     rs2::frameset frameset = pipe.wait_for_frames(1000);

//     if (rs2::motion_frame accel_frame = frameset.first_or_default(RS2_STREAM_ACCEL))
//     {
//       rs2_vector accel_sample = accel_frame.get_motion_data();
//       std::cout << "Accel:" << accel_sample.x << ", " << accel_sample.y << ", " << accel_sample.z << std::endl;
//     }

//     if (rs2::motion_frame gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO))
//     {
//       rs2_vector gyro_sample = gyro_frame.get_motion_data();
//       std::cout << "Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << std::endl;
//     }
//   }
// }

// TEST(realsense_test, imu_test_callback_D435i)
// {
//   auto callback = [&](const rs2::frame &frame)
//   {
//     if (frame.is<rs2::frameset>())
//     {
//       std::cout << "nonIMU data" << std::endl;
//     }
//     else
//     {
//       auto motion = frame.as<rs2::motion_frame>();
//       const auto& stream_type = motion.get_profile().stream_type();
//       std::cout << "gyro: " << (stream_type == RS2_STREAM_GYRO)
//                 << ", accel: " << (stream_type == RS2_STREAM_ACCEL) << std::endl;
//     }
//   };

//   rs2::pipeline pipe;

//   rs2::config rs_cfg_;
//   rs_cfg_.enable_stream(RS2_STREAM_ACCEL);
//   rs_cfg_.enable_stream(RS2_STREAM_GYRO);
//   rs_cfg_.enable_stream(RS2_STREAM_DEPTH); // , 256, 144, RS2_FORMAT_Z16, 90

//   std::this_thread::sleep_for(std::chrono::milliseconds(500));
//   pipe.start(rs_cfg_, callback);

//   std::cout << "RealSense callback sample" << std::endl
//             << std::endl;

//   while (true){}
// }

TEST(realsense_test, t260_and_d435)
{
  std::cout << "RealSense callback sample" << std::endl
            << std::endl;

  auto callback = [&](const rs2::frame &frame)
  {
    if (frame.is<rs2::frameset>())
    {
      std::cout << "nonIMU data" << std::endl;
    }
    else if (frame.is<rs2::motion_frame>())
    {
      auto motion = frame.as<rs2::motion_frame>();
      const auto &stream_type = motion.get_profile().stream_type();
      std::cout << "gyro: " << (stream_type == RS2_STREAM_GYRO)
                << ", accel: " << (stream_type == RS2_STREAM_ACCEL) << std::endl;
    }else if(frame.is<rs2::pose_frame>())
    {
      std::cout << "pose frame" << std::endl;
    }
  };

  rs2::context ctx;

  rs2::pipeline pipe_t265;
  rs2::config rs_cfg_t265;

  rs2::pipeline pipe_d435;
  rs2::config rs_cfg_d435;

  for (auto dev : ctx.query_devices())
  {
    std::cout << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
    if (strcmp(dev.get_info(RS2_CAMERA_INFO_NAME), "Intel RealSense D435") == 0)
    {
      rs_cfg_d435.enable_stream(RS2_STREAM_DEPTH); // , 256, 144, RS2_FORMAT_Z16, 90
    }
    else if (strcmp(dev.get_info(RS2_CAMERA_INFO_NAME), "Intel RealSense T265") == 0)
    {
      rs_cfg_t265.enable_stream(RS2_STREAM_ACCEL);
      rs_cfg_t265.enable_stream(RS2_STREAM_GYRO);
      rs_cfg_t265.enable_stream(RS2_STREAM_POSE);
    }
  }

  pipe_d435.start(rs_cfg_d435, callback);
  pipe_t265.start(rs_cfg_t265, callback);

  while (true)
  {
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}