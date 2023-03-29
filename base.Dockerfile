# https://www.docker.com/blog/faster-multi-platform-builds-dockerfile-cross-compilation-guide/

FROM nvcr.io/nvidia/cudagl:11.4.2-devel-ubuntu20.04 as amd64_base

FROM nvcr.io/nvidia/l4t-base:r32.3.1 as arm64_base

FROM ${TARGETARCH}_base as dev

# https://github.com/NVIDIA-AI-IOT/ros2_jetson/blob/main/docker/DockerFile.l4tbase.ros2.foxy
# https://docs.ros.org/en/foxy/Installation/Alternatives/Ubuntu-Development-Setup.html#id9
# https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

ARG ROS_PKG=ros_base
ENV ROS_DISTRO=foxy
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

ENV DEBIAN_FRONTEND=noninteractive


# change the locale from POSIX to UTF-8
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# add the ROS deb repo to the apt sources list
RUN apt-get upgrade -y \
    && apt-get install -y \
        curl \
        wget \
        gnupg2 \
        lsb-release \
        software-properties-common \
        python3-pip \
        terminator
    # && rm -rf /var/lib/apt/lists/* && apt-get clean

RUN curl \
    -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN add-apt-repository universe \
    && apt update \
    && apt upgrade -y \
    && apt install -y \
        ros-foxy-desktop \
        python3-argcomplete \
        ros-dev-tools \
    && apt-get install -y python3-rosdep

RUN rosdep init
RUN rosdep update
ENV RTI_NC_LICENSE_ACCEPTED=yes
RUN rosdep install --from-paths ${ROS_ROOT}/share --ignore-src -y --skip-keys "cyclonedds fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"


RUN echo "source /opt/ros/foxy/setup.bash" >> /etc/bash.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /etc/bash.bashrc

RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1

RUN apt-get install -y \
    libevdev-dev \
    libudev-dev \
    jstest-gtk \
    ros-${ROS_DISTRO}-rqt* \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-turtlebot3* \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-ros2-control

RUN mkdir -p /home/root/ros2_ws/src && mkdir /home/root/ros2_ws_tutorial
WORKDIR /home/root/ros2_ws