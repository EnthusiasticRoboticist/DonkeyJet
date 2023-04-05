NOT WORKING. GIVING UP


```bash
docker buildx use default

docker run --rm \
  --name=remote-buildkitd \
  --privileged \
  --gpus all \
  --runtime nvidia \
  --network host \
  -v /usr/bin/nvidia-container-runtime:/usr/bin/nvidia-container-runtime \
  -v /usr/bin/nvidia-container-runtime-hook:/usr/bin/nvidia-container-runtime-hook \
  -v /usr/share/lintian/overrides/nvidia-container-toolkit:/usr/share/lintian/overrides/nvidia-container-toolkit \
  -v /etc/nvidia-container-runtime:/etc/nvidia-container-runtime \
  -v /usr/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu \
  -v /usr/local/cuda-11.5:/usr/local/cuda-11.5 \
  -v /run/containerd/containerd.sock:/run/containerd/containerd.sock \
  -v /var/run/docker.sock:/var/run/docker.sock \
  moby/buildkit:latest \
  --addr tcp://0.0.0.0:1234 \
  --oci-worker-binary=/usr/bin/nvidia-container-runtime \
  --allow-insecure-entitlement security.insecure

docker exec -it remote-buildkitd sh
ln -s /usr/bin/buildkit-runc /usr/bin/runc
ln -s /usr/local/cuda-11.5 /usr/local/cuda

# on a new terminal (probably only once)
docker buildx create \
  --use \
  --name remote-container \
  --driver remote \
  tcp://localhost:1234

# on a new terminal
docker buildx build -f Dockerfile . --no-cache --progress plain
```
