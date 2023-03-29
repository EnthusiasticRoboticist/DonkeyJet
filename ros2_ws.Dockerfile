ARG BASE_IMAGE

FROM ${BASE_IMAGE} as BUILDER

COPY ros2_ws/src /home/root/ros2_ws/src
WORKDIR /home/root/ros2_ws 

# to make the "source" work
SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/foxy/setup.bash \
    && colcon build

RUN echo "source /home/root/ros2_ws/install/setup.bash" >> /home/root/.bashrc