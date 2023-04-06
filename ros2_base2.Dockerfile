# https://www.docker.com/blog/faster-multi-platform-builds-dockerfile-cross-compilation-guide/

FROM nvcr.io/nvidia/cudagl:11.4.2-devel-ubuntu18.04 as amd64_base

FROM nvcr.io/nvidia/l4t-base:r32.5.0 as arm64_base

FROM ${TARGETARCH}_base as dev

# https://github.com/NVIDIA-AI-IOT/ros2_jetson/blob/main/docker/DockerFile.l4tbase.ros2.foxy
# https://docs.ros.org/en/foxy/Installation/Alternatives/Ubuntu-Development-Setup.html#id9
# https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

ARG ROS_PKG=all
ENV ROS_DISTRO=foxy
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"] 

ENV DEBIAN_FRONTEND=noninteractive
ENV RTI_NC_LICENSE_ACCEPTED=yes

# change the locale from POSIX to UTF-8
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

RUN apt update \
  && apt upgrade -y \
  && apt install -y \
    software-properties-common \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    software-properties-common \
    terminator \
    build-essential \
    cmake \
    git \
    python3 \
    libpython3-dev \
  && add-apt-repository universe

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1

RUN apt update && apt install -y \
  libbullet-dev \
  python3-pip \
  python3-pytest-cov \
  # install some pip packages needed for testing
  && python3 -m pip install -U \
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    pytest \
    # install Fast-RTPS dependencies
  && apt install --no-install-recommends -y \
    libasio-dev \
    libtinyxml2-dev \
    # install Cyclone DDS dependencies
    libcunit1-dev
  
RUN apt install -y \
  python3-colcon-common-extensions \
  python3-numpy \
  python3-rosdep \
  python3-vcstool \
  python3-rosinstall-generator \
  && python -m pip install pip install setuptools==58.2.0


# get ROS2 code
RUN mkdir -p ${ROS_ROOT}/src \
  && cd ${ROS_ROOT} \
  && vcs import --input https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos src

RUN python3 -m pip install --upgrade pip && python3 -m pip install --upgrade --no-cache-dir --verbose cmake
RUN cmake --version

RUN echo 'export ROS_PACKAGE_PATH="${ROS_ROOT}/src"' > /setup_ROS_PACKAGE_PATH.sh \
  && echo 'for dir in ${ROS_ROOT}/src/*; do export ROS_PACKAGE_PATH="$dir:$ROS_PACKAGE_PATH"; done' >> /setup_ROS_PACKAGE_PATH.sh \
  && echo "source /setup_ROS_PACKAGE_PATH.sh >> /etc/bash.bashrc"

RUN cd ${ROS_ROOT} && source /setup_ROS_PACKAGE_PATH.sh \
  && apt upgrade -y \
  && rosdep init \
  && rosdep update \
  && rosinstall_generator desktop --rosdistro ${ROS_DISTRO} --deps --exclude RPP | vcs import src \
  && rosdep install -y -r --ignore-src --from-paths src --rosdistro ${ROS_DISTRO}

RUN cd ${ROS_ROOT} \
  && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN . ${ROS_ROOT}/install/local_setup.bash \
  && echo "source $ROS_ROOT/install/setup.bash" >> /etc/bash.bashrc \
  && echo "source $ROS_ROOT/install/local_setup.bash" >> /etc/bash.bashrc

RUN mkdir -p /root/ros2_ws/src && mkdir /root/ros2_ws_tutorial
WORKDIR /root/ros2_ws
