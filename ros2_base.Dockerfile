# https://www.docker.com/blog/faster-multi-platform-builds-dockerfile-cross-compilation-guide/

FROM nvcr.io/nvidia/cudagl:11.4.2-devel-ubuntu18.04 as amd64_base

FROM nvcr.io/nvidia/l4t-base:r32.5.0 as arm64_base

FROM ${TARGETARCH}_base as dev

# https://github.com/NVIDIA-AI-IOT/ros2_jetson/blob/main/docker/DockerFile.l4tbase.ros2.foxy
# https://docs.ros.org/en/foxy/Installation/Alternatives/Ubuntu-Development-Setup.html#id9
# https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

ARG ROS_PKG=desktop
ENV ROS_DISTRO=foxy
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"] 

ENV DEBIAN_FRONTEND=noninteractive


# change the locale from POSIX to UTF-8
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8


# add the ROS deb repo to the apt sources list
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
  curl \
  wget \
  gnupg2 \
  lsb-release \
  software-properties-common \
  terminator \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3 \
  libpython3-dev

RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1

RUN curl \
  -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get upgrade -y && apt-get install -y \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-numpy \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  python3-rosinstall-generator \
  libasio-dev \
  libtinyxml2-dev \
  libcunit1-dev \
  && add-apt-repository universe

RUN python3 -m pip install -U \
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
  pytest 

RUN python3 -m pip install --upgrade pip && python3 -m pip install --upgrade --no-cache-dir --verbose cmake
RUN cmake --version

ENV RTI_NC_LICENSE_ACCEPTED=yes

# https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/
RUN python -m pip install setuptools==58.2.0\
  && python -c "import setuptools; print(setuptools.__version__, 'HEYEHEYHEYHEY')" \
  && mkdir -p ${ROS_ROOT}/src \
  && cd ${ROS_ROOT} \
  && rosinstall_generator --deps --rosdistro ${ROS_DISTRO} ${ROS_PKG} \
		launch_xml \
		launch_yaml \
		launch_testing \
		launch_testing_ament_cmake \
		demo_nodes_cpp \
		demo_nodes_py \
		example_interfaces \
		camera_calibration_parsers \
		camera_info_manager \
		cv_bridge \
		v4l2_camera \
		vision_opencv \
		vision_msgs \
		image_geometry \
		image_pipeline \
		image_transport \
		compressed_image_transport \
		compressed_depth_image_transport \
		> ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall \
    && cat ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall \
    && vcs import src < ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall \
  # https://github.com/dusty-nv/jetson-containers/issues/181
  && rm -r ${ROS_ROOT}/src/ament_cmake \
  && git -C ${ROS_ROOT}/src/ clone https://github.com/ament/ament_cmake -b ${ROS_DISTRO} \
  # install dependencies using rosdep
  && apt-get update \
  && cd ${ROS_ROOT} \
  && rosdep init  \
  && rosdep update \
  && rosdep install -y \
      --ignore-src \
      --from-paths src \
      --rosdistro ${ROS_DISTRO} \
  && colcon build \
    --merge-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release 

RUN echo "source $ROS_ROOT/install/setup.bash" >> /etc/bash.bashrc

# RUN apt-get install -y \
#   libevdev-dev \
#   libudev-dev \
#   jstest-gtk \
#   ros-${ROS_DISTRO}-rqt* \
#   ros-${ROS_DISTRO}-navigation2 \
#   ros-${ROS_DISTRO}-nav2-bringup \
#   ros-${ROS_DISTRO}-turtlebot3* \
#   ros-${ROS_DISTRO}-joint-state-publisher-gui \
#   ros-${ROS_DISTRO}-robot-localization \
#   ros-${ROS_DISTRO}-xacro \
#   ros-${ROS_DISTRO}-ros2-control

RUN mkdir -p /root/ros2_ws/src && mkdir /root/ros2_ws_tutorial
WORKDIR /root/ros2_ws
