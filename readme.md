# Workspace for DonkeyJet

Introducing my ongoing robotics project, currently named **DonkeyJet**, inspired by the [Donkey Car project](https://www.donkeycar.com/). This mobile robot is built on an RC car frame and powered by Jetson Nano, providing a hands-on platform for practicing robotics knowledge. The goal is to continually enhance its capabilities while exploring various aspects of robotics technology. While I am not certain about the name DonkeyJet, I invite you to follow me on this exciting journey of discovery as I document my progress and share insights with fellow enthusiasts in my weblog [here](https://www.enthusiasticroboticist.com/).

## Docker Images

We are using docker for this project. [Read more here](https://www.enthusiasticroboticist.com/blog/ros-2-on-jetson-nano-using-docker/).

- [setup remote docker build](https://youtu.be/YX2BSioWyhI)
- add Jetson IP to `/etc/hosts` as `jetson`
- some docker docs
  - https://docs.docker.com/build/building/multi-platform/
- https://www.docker.com/blog/multi-arch-build-and-images-the-simple-way/

Setup for building docker image for multiple platforms which images are built natively on the corresponding hardware without emulation.

```bash
export REGISTRY="<the docker registry to be used, or dockerhub user>"

docker context create localamd64 --docker  "host=unix:///var/run/docker.sock"
docker context create jetson --docker "host=tcp://jetson:2375"

# just one time
docker buildx create --use  --driver-opt network=host --config=buildkitd.toml --name MultiPlatform
docker buildx create --use  --driver-opt network=host --config=buildkitd.toml --name mybuilder localamd64
docker buildx create --append --driver-opt network=host --config=buildkitd.toml --name mybuilder jetson
```

### Docker image with CUDA and ROS2

```bash
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -f Dockerfile.base \
  -t ${REGISTRY}/ros_base:latest \
  --push .


# test the images
docker context use localamd64
docker pull ${REGISTRY}/ros_base:latest
docker run \
  --name ros_base \
  --rm \
  -it \
  --runtime nvidia \
  --network host \
  --gpus all \
  -e DISPLAY \
  ${REGISTRY}/ros_base:latest \
  bash
```

## ROS2 Workspace Docker Container

```bash
# build my image
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -f Dockerfile.ros2_ws \
  --build-arg BASE_IMAGE=${REGISTRY}/ros_base:latest \
  -t ${REGISTRY}/ros2_ws:latest \
  --push .
```

## Docker For Development
```bash
export JETSON_IP=192.168.68.68

function BUILD_AND_RUN(){
  docker -H $JETSON_IP pull ${REGISTRY}/ros_base:latest
  docker -H $JETSON_IP buildx build -f Dockerfile.ros2_ws  --build-arg BASE_IMAGE=${REGISTRY}/ros_base:latest   -t ${REGISTRY}/ros2_ws:latest .
  docker -H $JETSON_IP run --name ros2_ws --rm -it --runtime nvidia --network host --gpus all --privileged -e DISPLAY -v /dev:/dev -v /proc:/proc -v /sys:/sys ${REGISTRY}/ros2_ws:latest bash -ic "$@"
}
export EXEC_RUN="docker -H $JETSON_IP exec -it ros2_ws /bin/bash -ic"

# Run nodes separately
BUILD_AND_RUN "ros2 run bot_hardware joy"
$EXEC_RUN "ros2 run bot_hardware pca9685"

# run the launch file
BUILD_AND_RUN "ros2 launch bot_hardware manual_control_launch.py"

# local dev env
docker run \
  --name ros_base \
  --rm \
  -it \
  --runtime nvidia \
  --network host \
  --gpus all \
  --privileged \
  -e DISPLAY \
  -v /dev:/dev \
  -v /proc:/proc \
  -v /sys:/sys \
  -v `pwd`/ros2_ws/src:/home/dev/ros2_ws/src \
  -v `pwd`/ros2_ws_tutorial/src:/home/dev/ros2_ws_tutorial/src \
  ${REGISTRY}/ros_base:latest \
  bash 
```

### Docker image with CUDA, ROS2, Realsense
- https://dev.intelrealsense.com/docs/nvidia-jetson-tx2-installation


```bash
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -f Dockerfile.realsense \
  --build-arg BASE_IMAGE=${REGISTRY}/ros_base:latest \
  -t ${REGISTRY}/ros_realsense:latest \
  --push .

# on jetson run only once
sudo curl https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules -o /etc/udev/rules.d/99-realsense-libusb.rules \
  && sudo udevadm control --reload-rules \
  && sudo udevadm trigger

docker run \
  --name ros_realsense \
  --rm \
  -it \
  --runtime nvidia \
  --network host \
  --gpus all \
  --privileged \
  -v /dev:/dev \
  -v /proc:/proc \
  -v /sys:/sys \
  ${REGISTRY}/ros_realsense:latest \
  bash


# to test run the following in the docker container
rs-depth

ros2 launch realsense2_camera rs_launch.py initial_reset:=true
```

## Joystick

[Read more](https://www.enthusiasticroboticist.com/blog/using-bluetooth-controller-with-ros-2-on-jetson-nano/)
[About Joystick in Linux](https://opencoursehub.cs.sfu.ca/bfraser/grav-cms/cmpt433/links/files/2022-student-howtos/LinuxJoystick.hLibrary.pdf).

### Test the joystick

With GUI
```bash
sudo apt install -y jstest-gtk
jstest-gtk
```

Without GUI
```bash
jstest /dev/input/js0
```

With ros
```
docker -H 192.168.68.68 pull ${REGISTRY}/ros_base:latest
docker -H 192.168.68.68 run \
  --rm -it  --runtime nvidia --network host \
  --gpus all --privileged \
  -v /dev:/dev -v /proc:/proc -v /sys:/sys \
  ${REGISTRY}/ros_base:latest \
  bash
```


## Run motors

[Read more here](https://www.enthusiasticroboticist.com/blog/actuation-and-pca9685-with-ros-2-on-jetson-nano/)

```bash
docker run \
  --name realsense
  --rm \
  -it \
  --runtime nvidia \
  --network host \
  --gpus all \
  --privileged \
  -v /dev:/dev \
  -v /proc:/proc \
  -v /sys:/sys \
  ${REGISTRY}/ros_realsense:latest \
  bash

# issue about Jetson.GPIO inside docker: 
# https://forums.developer.nvidia.com/t/jetson-gpio-not-working-on-python-inside-the-container/180435
python -m pip install adafruit-circuitpython-servokit Jetson.GPIO
i2cdetect -y -r 1
```

- [More about I2C in Linux](https://www.youtube.com/watch?v=-1PHQYRbAm8&ab_channel=Johannes4GNU_Linux)
- [PCA9685 datasheet](https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf)
- [PCA9685 Adafruit info](https://cdn-learn.adafruit.com/downloads/pdf/16-channel-pwm-servo-driver.pdf)
- The I2C address is `0x40`.
- [About Actutation](http://docs.donkeycar.com/parts/actuators/)
- [I2C in Linux](http://embeddedcraft.org/eclinux/linuxi2c.html)

### PWM Signals
- PWM Period: 20 ms, PWM Frequency: 50Hz
- 1ms - 2ms high or Duty cycle: 1% - 10% with 50Hz freq
- Servo from left to right
- Throttle form full backward to full forward.

```
freq = 60
prescaler = 0x66 = 25M / (4096 * frequency)
i2cset -y 1 0x40 0x00 0x90
i2cset -y 1 0x40 0xFE 0x66
i2cset -y 1 0x40 0x00 0xA0

# steering
i2cset -y 1 0x40 0x0A 0x00 && \
  i2cset -y 1 0x40 0x0B 0x00 && \
  i2cset -y 1 0x40 0x0C 0xFA && \
  i2cset -y 1 0x40 0x0D 0x00

i2cset -y 1 0x40 0x0A 0x00 0x00 0x4F 0x01 i

# throttle (85 instead of 6f is good value for test)
i2cset -y 1 0x40 0x06 0x00 && \
i2cset -y 1 0x40 0x07 0x00 && \
i2cset -y 1 0x40 0x08 0x85 && \
i2cset -y 1 0x40 0x09 0x01

i2cset -y 1 0x40 0x06 0x00 0x00 0x4F 0x01 i

```
