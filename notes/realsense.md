# Docker image with CUDA, ROS2, Realsense
- https://dev.intelrealsense.com/docs/nvidia-jetson-tx2-installation


```bash
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -f realsense.Dockerfile \
  --build-arg BASE_IMAGE=${REGISTRY}/ros2_base2:latest \
  -t ${REGISTRY}/ros2_realsense:latest \
  --push .

# on jetson run only once
sudo curl https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules -o /etc/udev/rules.d/99-realsense-libusb.rules \
  && sudo udevadm control --reload-rules \
  && sudo udevadm trigger

docker run \
  --name ros2_realsense \
  --rm \
  -it \
  --runtime nvidia \
  --network host \
  --gpus all \
  --privileged \
  -v /dev/bus/usb/:/dev/bus/usb/ \
  -e DISPLAY \
  ${REGISTRY}/ros2_realsense:latest \
  bash

# to test run the following in the docker container
rs-depth

ros2 launch realsense2_camera rs_launch.py initial_reset:=true
```

# Prepare IMU data from D435i for ROS 2

```bash
MOUNT_WS
cd ws/donkeyJet

rs-enumerate-devices  # check the device
# once
sudo udevadm control --reload-rules
udevadm trigger

docker pull ${REGISTRY}/ros2_realsense:latest

#run this to load software to T265
docker run \
  --name ros2_base \
  --rm \
  --privileged \
  -v /dev/bus/usb/:/dev/bus/usb/ \
  ${REGISTRY}/ros2_realsense_latest:latest \
  rs-pose

# for development on host:
docker run \
  --name ros2_base \
  --rm \
  -it \
  --runtime nvidia \
  --network host \
  --gpus all \
  --privileged \
  -e DISPLAY \
  -v /dev/bus/usb/:/dev/bus/usb/ \
  -v `pwd`/ros2_ws/src:/root/ros2_ws/src \
  -v `pwd`/ros2_ws_tutorial/src:/root/ros2_ws_tutorial/src \
  ${REGISTRY}/ros2_realsense:latest \
  bash 

# ${REGISTRY}/ros2_realsense:latest \

colcon build --symlink-install

# for test on jetson:
export JETSON_IP=
export HOST=""
export HOST="-H $JETSON_IP"
function RUN(){
  docker $HOST run --name ros2_ws --rm -it \
    --runtime nvidia --network host \
    --gpus all -e DISPLAY \
    --privileged -v /proc:/proc -v /dev:/dev -v /sys:/sys \
    ${REGISTRY}/ros2_ws:latest bash -ic "$@"
}

function BUILD_AND_RUN(){
  docker $HOST pull ${REGISTRY}/ros2_realsense:latest && \
  docker $HOST buildx build -f ros2_ws.Dockerfile  --build-arg BASE_IMAGE=${REGISTRY}/ros2_realsense:latest   -t ${REGISTRY}/ros2_ws:latest . && \
  RUN "$@"
}



ros2 run bot_hardware realsense
BUILD_AND_RUN "ros2 run bot_hardware realsense --ros-args --log-level debug" # --ros-args --log-level debug

ros2 run tf2_tools view_frames.py
ros2 launch realsense2_camera rs_launch.py
ros2 run tf2_ros tf2_echo camera_pose_frame odom_frame

reset; colcon build && \
  ./build/bot_hardware/bot_hardware_test --gtest_filter="realsense_lib.*"
```

# build realsense image
```bash
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -f realsense.Dockerfile \
  --build-arg BASE_IMAGE=${REGISTRY}/ros2_base:latest \
  -t ${REGISTRY}/ros2_realsense:latest \
  --push .
```

## Expected IMU type

- More information about message type can be found [here](https:docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html).
- Example of including librealsense SKD into a [ros2 CMakeLists.txt file](https:github.com/IntelRealSense/realsense-ros/blob/ros2-development/realsense2_camera/CMakeLists.txt)
- Realsense How-to [wiki page](https:github.com/IntelRealSense/librealsense/wiki/API-How-To).
- Realsense cpp [IMU example](https:github.com/GruffyPuffy/imutest/blob/master/imutest.cpp).


# Notes

- You need to plug the USB first, then run the container. When you re-plug the USB, you need to kill the docker container and re-run it. Otherwise, realsense device would not be visible within the container.
- Sometimes the IMU streams are not coming. I don't know why. Re-plugging could help.
- reading imu info
- T265 doesn't have memory for firmware and usb volume bind and container issues

```cpp
rs2::pipeline pipe;
rs2::config cfg;
cfg.enable_stream(RS2_STREAM_ACCEL);
cfg.enable_stream(RS2_STREAM_GYRO);
cfg.enable_stream(RS2_STREAM_DEPTH);
pipe.start(cfg);

while (true)
{
  rs2::frameset frameset = pipe.wait_for_frames(1000);

  if (rs2::motion_frame accel_frame = frameset.first_or_default(RS2_STREAM_ACCEL))
  {
    rs2_vector accel_sample = accel_frame.get_motion_data();
    std::cout << "Accel:" << accel_sample.x << ", " << accel_sample.y << ", " << accel_sample.z << std::endl;
  }

  if (rs2::motion_frame gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO))
  {
    rs2_vector gyro_sample = gyro_frame.get_motion_data();
    std::cout << "Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << std::endl;
  }
}

auto callback = [&](const rs2::frame &frame)
{
  if (frame.is<rs2::frameset>())
  {
    std::cout << "Depth data" << std::endl;
  } else if (frame.is<rs2::motion_frame>())
  {
    auto motion = frame.as<rs2::motion_frame>();
    const auto &stream_type = motion.get_profile().stream_type();
    std::cout << "gyro: " << (stream_type == RS2_STREAM_GYRO)
              << ", accel: " << (stream_type == RS2_STREAM_ACCEL) << std::endl;
  } else if(frame.is<rs2::pose_frame>())
  {
    std::cout << "pose frame" << std::endl;
  }
};
```

```cpp
#include <librealsense2/rs.hpp>
#include <iostream>

int main()
{
  rs2::pipeline pipe;
  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_ACCEL);
  cfg.enable_stream(RS2_STREAM_GYRO);
  pipe.start(cfg);

  while (true) // Application still alive?
  {
    rs2::frameset frameset = pipe.wait_for_frames(1000);

    if (rs2::motion_frame accel_frame = frameset.first_or_default(RS2_STREAM_ACCEL))
    {
      rs2_vector accel_sample = accel_frame.get_motion_data();
      std::cout << "Accel:" << accel_sample.x << ", " << accel_sample.y << ", " << accel_sample.z << std::endl;
    }

    if (rs2::motion_frame gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO))
    {
      rs2_vector gyro_sample = gyro_frame.get_motion_data();
      std::cout << "Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << std::endl;
    }
  }
  return 0;
}
```

```cpp
#include <librealsense2/rs.hpp>
#include <iostream>

int main(){
  auto callback = [&](const rs2::frame &frame)
  {
    if (frame.is<rs2::frameset>())
    {
      std::cout << "Depth data" << std::endl;
    } else if (frame.is<rs2::motion_frame>())
    {
      auto motion = frame.as<rs2::motion_frame>();
      const auto &stream_type = motion.get_profile().stream_type();
      std::cout << "gyro: " << (stream_type == RS2_STREAM_GYRO)
                << ", accel: " << (stream_type == RS2_STREAM_ACCEL) << std::endl;
    } else if(frame.is<rs2::pose_frame>())
    {
      std::cout << "pose frame" << std::endl;
    }
  };

  rs2::pipeline pipe;
  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_ACCEL);
  cfg.enable_stream(RS2_STREAM_GYRO);
  cfg.enable_stream(RS2_STREAM_DEPTH); 

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  pipe.start(cfg, callback);
  while (true){}
  return 0;
}
```

```cpp
{
  auto callback = [&](const rs2::frame &frame)
  {
    // callback code as before
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
```