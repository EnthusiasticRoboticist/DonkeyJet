

```bash
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
  ${REGISTRY}/ros2_realsense_latest:latest \
  bash 


docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -f rtabmap.Dockerfile \
  --build-arg REGISTRY=${REGISTRY} \
  -t ${REGISTRY}/rtabmap:latest \
  --push .

```


```ANSI
ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
rtabmap_slam: No definition of [nav2_msgs] for OS version [bionic]
rtabmap_examples: No definition of [velodyne] for OS version [bionic]
rtabmap_util: No definition of [pcl_ros] for OS version [bionic]
rtabmap: No definition of [octomap] for OS version [bionic]
realsense2_camera: No definition of [librealsense2] for OS version [bionic]
rtabmap_demos: No definition of [nav2_bringup] for OS version [bionic]
rtabmap_odom: No definition of [pcl_ros] for OS version [bionic]
realsense2_description: No definition of [xacro] for OS version [bionic]
Continuing to install resolvable dependencies...
```


pcl
```bash
apt remove -y libpcl-dev
apt autoremove
cd /root

# dependencies
apt update --fix-missing && apt upgrade --fix-missing
# or libvtk6-dev 

apt install libflann-dev \
  libvtk7-dev \
  libvtk7-qt-dev \
  libpcap-dev \
  libboost-filesystem-dev \
  libboost-iostreams-dev \
  libboost-system-dev \
  libboost-date_time-dev

  libboost-all-dev \

wget https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.11.1.tar.gz
tar xvf pcl-1.11.1.tar.gz && rm pcl-1.11.1.tar.gz
cd pcl-pcl-1.11.1
mkdir build && cd build
cmake ..
make -j20 install

git clone https://github.com/ros-planning/navigation2.git src/navigation2
cd src/navigation2
git checkout foxy-devel
cd -
rosdep install --from-paths src --ignore-src -r -y --skip-keys=libpcl-dev
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to nav2_msgs

cd ros2_pre_installed
git clone https://github.com/ros-perception/perception_pcl.git src/perception_pcl
git checkout foxy-devel
cd -
rosdep install --from-paths src --ignore-src -r -y --skip-keys=libpcl-dev
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select pcl_ros

# from the dockerfile
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select-regex rtabmap*

```


```
reset; docker buildx build   --platform linux/amd64   -f ros2_base.Dockerfile   -t ros2_base2:latest   --load .
reset; docker buildx build   --platform linux/arm64   -f ros2_base.Dockerfile   -t ros2_base2:latest   --output=tar .
docker run --rm -it ros2_base2:latest bash


docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -f ros2_base2.Dockerfile \
  -t ${REGISTRY}/ros2_base2:latest \
  --push .

docker run \
  --name ros2_base \
  --rm \
  -it \
  --runtime nvidia \
  --network host \
  --gpus all \
  -e DISPLAY \
  ${REGISTRY}/ros2_base2:latest \
  bash
```