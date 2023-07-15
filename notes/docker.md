# Docker Images

We are using docker for this project. [Read more here](https://www.enthusiasticroboticist.com/blog/ros-2-on-jetson-nano-using-docker/).

- [setup remote docker build](https://youtu.be/YX2BSioWyhI)
- add Jetson IP to `/etc/hosts` as `jetson`
- some docker docs
  - https://docs.docker.com/build/building/multi-platform/
- https://www.docker.com/blog/multi-arch-build-and-images-the-simple-way/

Setup for building docker image for multiple platforms which images are built natively on the corresponding hardware without emulation.

```bash
/usr/bin/dockerd -H fd:// --containerd=/run/containerd/containerd.sock
export REGISTRY="<the docker registry to be used, or dockerhub user>"

docker context create jetson --docker "host=tcp://jetson:2375"
docker context create localamd64 --docker "host=unix:///var/run/docker.sock"

# just one time
# docker buildx create --use  --driver-opt network=host --config=buildkitd.toml --name MultiPlatform
docker buildx create --use --driver-opt network=host --config=buildkitd-arm64.toml --name mybuilder jetson
docker buildx create --append --driver-opt network=host --config=buildkitd-amd64.toml --name mybuilder localamd64
```
