# Prepare IMU data from D435i for ROS 2

```bash
MOUNT_WS
cd ws/donkeyJet

rs-enumerate-devices  # check the device
# once
sudo udevadm control --reload-rules
udevadm trigger

docker pull ${REGISTRY}/ros2_realsense:latest

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
  -v `pwd`/ros2_ws/src:/root/ros2_ws/src \
  -v `pwd`/ros2_ws_tutorial/src:/root/ros2_ws_tutorial/src \
  ${REGISTRY}/ros2_realsense_latest:latest \
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
```

## Expected IMU type

- More information about message type can be found [here](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html).
- Example of including librealsense SKD into a [ros2 CMakeLists.txt file](https://github.com/IntelRealSense/realsense-ros/blob/ros2-development/realsense2_camera/CMakeLists.txt)
- Realsense How-to [wiki page](https://github.com/IntelRealSense/librealsense/wiki/API-How-To).
- Realsense cpp [IMU example](https://github.com/GruffyPuffy/imutest/blob/master/imutest.cpp).


# Notes

- You need to plug the USB first, then run the container. When you re-plug the USB, you need to kill the docker container and re-run it. Otherwise, realsense device would not be visible within the container.
- Sometimes the IMU streams are not coming. I don't know why. Re-plugging could help.
- reading imu info
- 