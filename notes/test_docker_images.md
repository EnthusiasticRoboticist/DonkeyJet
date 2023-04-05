# Fixing Realsense issues

docker image

```bash
# On Jetson
MOUNT_WS

# initial_reset:=true
ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x6 rgb_camera.profile:=640x480x6

# log_level:=DEBUG 
# enable_color:=false
```

# CUDA test

```bash
# on host
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -f cudatest.Dockerfile \
  -t ${REGISTRY}/cudatest:latest \
  --build-arg BASE_IMAGE=${REGISTRY}/ros2_base2:latest \
  --push .

docker pull ${REGISTRY}/cudatest:latest
docker run --rm -it --runtime nvidia ${REGISTRY}/cudatest:latest 
```

# Experiment with remote buildkit

```bash
docker run --rm \
  --name=remote-buildkitd \
  --privileged \
  --gpus all \
  -p 1234:1234 \
  -v /usr/bin/nvidia-container-runtime:/usr/bin/nvidia-container-runtime \
  -v /etc/nvidia-container-runtime:/etc/nvidia-container-runtime \
  -v /usr/local:/usr/local \
  moby/buildkit:latest \
  --addr tcp://0.0.0.0:1234 \
  --oci-worker-binary=/usr/bin/nvidia-container-runtime

docker buildx create \
  --use \
  --name remote-container \
  --driver remote \
  tcp://${JETSON_IP}:1234

docker exec -it remote-buildkitd sh
ln -s /usr/bin/buildkit-runc /usr/bin/runc

docker buildx build \
  --platform linux/arm64 \
  -f cudatest.Dockerfile \
  -t ${REGISTRY}/cudatest:latest \
  --push .
```
