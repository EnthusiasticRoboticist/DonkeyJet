ARG BASE_IMAGE

FROM ${BASE_IMAGE} as BUILDER

COPY ros2_ws/src /root/ros2_ws/src
WORKDIR /root/ros2_ws 

# to make the "source" work
SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/foxy/install/setup.bash \
    && colcon build

RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc