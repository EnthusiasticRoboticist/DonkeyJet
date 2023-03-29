# Fixing Realsense issues

docker image

```bash
# On Jetson
MOUNT_WS

docker pull ${REGISTRY}/ros_base:latest

docker run \
  --name realsense_fixing \
  --rm \
  -it \
  --runtime nvidia \
  --network host \
  --gpus all \
  --privileged \
  -v /dev/input:/dev/input \
  -v /dev/v4l:/dev/v4l \
  -v `pwd`/thirdparty/librealsense:/home/dev/librealsense \
  -v `pwd`/thirdparty/realsense-ros:/home/dev/ros2_ws/src/realsense-ros \
  ${REGISTRY}/ros_base:latest \
  bash

  -v /proc:/proc \
  -v /sys:/sys \
  -v /dev:/dev \

sudo apt-get update \
  && sudo apt-get install -y \
      xorg-dev \
      libusb-1.0-0-dev \
      libxinerama-dev \
      python3 \
      python3-dev \
      libpython3.8-dev \
      gcc-8 g++-8 \
  && sudo rm /usr/bin/gcc /usr/bin/g++ \
  && sudo ln -s gcc-8 /usr/bin/gcc \
  && sudo ln -s g++-8 /usr/bin/g++


cd ~/librealsense
# on development branch before removing T265 stuff
git checkout 95a824a8d6ff49161fa28605994190de49e62f73

cd ~/librealsense/build \
  && cmake \
    -DCMAKE_BUILD_TYPE=release \
    -DBUILD_EXAMPLES=true \
    -DFORCE_RSUSB_BACKEND=ON \
    -DBUILD_WITH_CUDA=true \
    -DBUILD_PYTHON_BINDINGS=bool:true \
    -DPYBIND11_INSTALL=ON \
    -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
    -DPYTHON_EXECUTABLE:FILEPATH=$(python3 -c "import sys; print(sys.executable)") \
    -DPYTHON_INCLUDE_DIR:PATH=$(python3 -c "import sysconfig; print(sysconfig.get_path('include'))") \
    -DPYTHON_LIBRARY:FILEPATH=$(python3 -c "import sysconfig; import glob; print(glob.glob('/*/'.join(sysconfig.get_config_vars('LIBDIR', 'INSTSONAME')))[0])") \
    .. \
  && make -j`nproc` install

cd /home/dev/ros2_ws/src/realsense-ros

udevadm control --reload-rules && udevadm trigger
docker commit realsense_fixing ${REGISTRY}/ros_realsense:fixing

cd ~/ws/donkeyJet

docker run \
  --name realsense_fixing \
  --rm \
  -it \
  --runtime nvidia \
  --network host \
  --gpus all \
  --privileged \
  -v /dev:/dev \
  -v /proc:/proc \
  -v /sys:/sys \
  -v `pwd`/thirdparty/librealsense:/home/dev/librealsense \
  -v `pwd`/thirdparty/realsense-ros:/home/dev/ros2_ws/src/realsense-ros \
  ${REGISTRY}/ros_realsense:fixing \
  bash

# initial_reset:=true
ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x6 rgb_camera.profile:=640x480x6

log_level:=DEBUG 
enable_color:=false

```

```bash
git clone https://gitlab.com/nvidia/container-images/l4t-jetpack.git
cd l4t-jetpack/
cat /etc/nv_tegra_release
# R32 (release), REVISION: 7.3, GCID: 31982016, BOARD: t210ref, EABI: aarch64, DATE: Tue Nov 22 17:30:08 UTC 2022

sudo make image TAG=r32.7.1

docker run -it --rm --runtime nvidia --gpus all --privileged nvcr.io/nvidia/l4t-base:r32.7.1 bash

```







# CUDA test

```bash
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -f cudatest.Dockerfile \
  -t ${REGISTRY}/cudatest:latest \
  --push .

docker pull ${REGISTRY}/cudatest:latest
docker run --pull --rm -it --runtime nvidia ${REGISTRY}/cudatest:latest 
```





# Firmware
Not working:
- T265: https://realsense-hw-public.s3.amazonaws.com/Releases/TM2/FW/target/0.2.0.926



