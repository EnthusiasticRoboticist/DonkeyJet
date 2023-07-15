# ROS 2


## Humble

```bash
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -f ros2_humble.Dockerfile \
  -t ${REGISTRY}/ros2_humble:latest \
  --push .

docker pull ${REGISTRY}/ros2_humble:latest

docker run \
  --name humble \
  --rm \
  -it \
  --runtime nvidia \
  --network host \
  --gpus all \
  -e DISPLAY \
  ${REGISTRY}/ros2_humble:latest \
  bash


colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to-regex python_orocos_kdl_vendor
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to-regex qt_gui_cpp python_orocos_kdl_vendor
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select qt_gui_cpp python_orocos_kdl_vendor

update-alternatives --remove python /usr/bin/python3.9
update-alternatives --remove python3 /usr/bin/python3.9


# RUN python -m pip install -U \
#     PyQt5 \
#   && apt install --no-install-recommends -y \
#     sip-dev pyqt5-dev \
#     python3-apt \
#   && python3.6 /usr/bin/add-apt-repository -y ppa:beineri/opt-qt-5.15.2-bionic \
#   && apt update \
#   && apt install 
```


## Foxy

### Docker image with CUDA and ROS2

```bash
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -f ros2_base.Dockerfile \
  -t ${REGISTRY}/ros2_base2:latest \
  --push .

docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -f ros2_base.Dockerfile \
  -t ${REGISTRY}/ros2_base2:latest \
  --output=tar .

# test the images
docker pull ${REGISTRY}/ros2_base2:latest
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

# test images
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener
```


## ROS2 Workspace Docker Container

```bash
# build my image
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -f ros2_ws.Dockerfile \
  --build-arg BASE_IMAGE=${REGISTRY}/ros2_base2:latest \
  -t ${REGISTRY}/ros2_ws:latest \
  --push .


docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -f ros2_ws.Dockerfile \
  --build-arg BASE_IMAGE=${REGISTRY}/ros2_base2:latest \
  -t ${REGISTRY}/ros2_ws:latest \
  . --output registry.insecure=true,push=true,type=image
```

## Docker For Development
```bash
export JETSON_IP=

function BUILD_AND_RUN(){
  docker -H $JETSON_IP pull ${REGISTRY}/ros2_base2:latest
  docker -H $JETSON_IP buildx build -f ros2_ws.Dockerfile  --build-arg BASE_IMAGE=${REGISTRY}/ros2_base2:latest   -t ${REGISTRY}/ros2_ws:latest .
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
  ${REGISTRY}/ros2_base2:latest \
  bash 
```

