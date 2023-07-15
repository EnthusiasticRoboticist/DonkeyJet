# https://www.docker.com/blog/faster-multi-platform-builds-dockerfile-cross-compilation-guide/

FROM nvcr.io/nvidia/cudagl:11.4.2-devel-ubuntu18.04 as amd64_base

FROM nvcr.io/nvidia/l4t-base:r32.5.0 as arm64_base

FROM ${TARGETARCH}_base as dev

SHELL ["/bin/bash", "-c"] 
ENV SKIP_ROSDEP=""

ENV ROS_PYTHON_VERSION=3
ENV SHELL /bin/bash

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
    && apt install -y \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    terminator \
    build-essential \
    cmake \
    git \
    xorg-dev \
    libusb-1.0-0-dev \
    libxinerama-dev \
    gcc-8 g++-8 \
    && rm /usr/bin/gcc /usr/bin/g++ \
    && ln -s gcc-8 /usr/bin/gcc \
    && ln -s g++-8 /usr/bin/g++ \
    && add-apt-repository universe
    
    # && add-apt-repository -y ppa:deadsnakes/ppa \
    # python3.9 \
    # libpython3.9-dev \

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN update-alternatives --install /usr/bin/python python /usr/bin/python3.6 1 \
  && update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.6 1

RUN apt update && apt install -y \
  libbullet-dev \
  python3-pip \
  python3-distutils \
  python3-pytest-cov
  # python3.9-distutils \
RUN python -m pip install -U setuptools pip distlib
RUN python -m pip install -U \
  argcomplete \
  colcon-common-extensions \
  numpy \
  rosdep \
  vcstool \
  rosinstall-generator \
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
  pytest
# install Fast-RTPS dependencies
RUN apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev \
  # install Cyclone DDS dependencies
  libcunit1-dev

# PCL 1.12 from source. PCL 1.8 from apt has some issues
# which we face with rtab_map

## Dependencies
RUN apt install -y libflann-dev \
  libvtk6-dev \
  libvtk6-qt-dev \
  libpcap-dev \
  libboost-filesystem-dev \
  libboost-iostreams-dev \
  libboost-system-dev \
  libboost-date-time-dev

## PCL 1.12.1 installation from source
RUN wget https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.12.1.tar.gz \
  && tar xvf pcl-1.12.1.tar.gz && rm pcl-1.12.1.tar.gz \
  && cd pcl-pcl-1.12.1 \
  && mkdir build && cd build \
  && cmake .. \
  && make -j`nproc` install

RUN python -m pip install --upgrade pip && python -m pip install --upgrade --no-cache-dir --verbose cmake

# Should be Space seperated stirng
ENV ROSDEP_SKIP_PACKAGES="fastcdr rti-connext-dds-6.0.1 urdfdom_headers libpcl-dev"
ENV ROS_DISTRO=humble
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

# get ROS2 code
RUN mkdir -p ${ROS_ROOT}/src \
  && cd ${ROS_ROOT} \
  && vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

ENV PYBIND11_PYTHON_VERSION=3.6

# RUN cd ${ROS_ROOT} \
#   && rosdep init \
#   && rosdep update \
#   && rosdep install --from-paths src --ignore-src -r -y --rosdistro=$ROS_DISTRO --skip-keys="$ROSDEP_SKIP_PACKAGES"

# RUN cd ${ROS_ROOT} \
#   && export PYBIND11_PYTHON_VERSION=3.6 \
#   && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# RUN source $ROS_ROOT/install/setup.bash \
#   && rosinstall_generator desktop --rosdistro ${ROS_DISTRO} --deps --exclude RPP | vcs import src

# RUN . ${ROS_ROOT}/install/setup.bash \
#   && echo "source $ROS_ROOT/install/setup.bash" >> /etc/bash.bashrc

# RUN TEST_PLUGINLIB_PACKAGE="${ROS_ROOT}/build/pluginlib/pluginlib_enable_plugin_testing/install/test_pluginlib__test_pluginlib/share/test_pluginlib/package.xml" && \
#   sed -i '/<\/description>/a <license>BSD<\/license>' $TEST_PLUGINLIB_PACKAGE && \
#   sed -i '/<\/description>/a <maintainer email="michael@openrobotics.org">Michael Carroll<\/maintainer>' $TEST_PLUGINLIB_PACKAGE && \
#   cat $TEST_PLUGINLIB_PACKAGE

# # To install ROS packages from source
# RUN mkdir -p /root/ros2_pre_installed/src

# #ompl
# RUN git clone https://github.com/ompl/ompl.git \
#   && cd ompl \
#   && mkdir build && cd build \
#   && cmake .. && make -j`nproc` install

# # OctoMap
# ENV ROSDEP_SKIP_PACKAGES="$ROSDEP_SKIP_PACKAGES liboctomap"
# RUN git clone https://github.com/OctoMap/octomap.git \
#   && cd octomap/octomap \
#   && mkdir build && cd build && cmake .. && make -j`nproc` install

# # pcl_ros
# # BehaviorTree.CPP
# # gazebo_ros_pkgs
# # navigation2
# # OctoMap
# # rtab-map
# RUN cd /root/ros2_pre_installed \
#   && git clone https://github.com/ros-perception/perception_pcl.git src/perception_pcl \
#   && cd src/perception_pcl \
#   && git checkout foxy-devel \
#   && cd - \
#   && git clone -b debian/foxy/behaviortree_cpp_v3 https://github.com/BehaviorTree/behaviortree_cpp_v3-release.git src/behaviortree_cpp_v3-release \
#   && curl -sSL http://get.gazebosim.org | sh \
#   && git clone -b humble https://github.com/ros-perception/image_common.git src/image_common \
#   && git clone -b debian/humble/gazebo_ros https://github.com/ros2-gbp/gazebo_ros_pkgs-release.git src/gazebo_ros_pkgs \
#   && git clone -b humble https://github.com/ros-planning/navigation2.git src/navigation2 \
#   && git clone -b ros2 https://github.com/OctoMap/octomap_msgs.git src/octomap_msgs \
#   && git clone -b ros2 https://github.com/OctoMap/octomap_ros.git src/octomap_ros \
#   && git clone -b humble-devel https://github.com/introlab/rtabmap.git src/rtabmap \
#   && git clone -b humble-devel https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros \
#   && grep -l -r '<octomap_msgs\/conversions.h>' | xargs sed -i "s/<octomap_msgs\/conversions.h>/<octomap_msgs\/octomap_msgs\/conversions.h>/g" \
#   && cd - \
#   && source $ROS_ROOT/install/setup.bash \
#   && rosdep install --from-paths src --ignore-src -r -y --rosdistro=$ROS_DISTRO --skip-keys="$ROSDEP_SKIP_PACKAGES" \
#   && colcon build --merge-install --install-base "$ROS_ROOT/install" --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to-regex pcl_ros nav2* rtabmap*

# # behaviortree_cpp_v3 gazebo* nav2* navigation2 smac_planner octomap_msgs octomap_ros rtabmap*

# RUN mkdir -p /root/ros2_ws/src && mkdir /root/ros2_ws_tutorial
# WORKDIR /root/ros2_ws
