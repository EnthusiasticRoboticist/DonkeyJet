ARG BASE_IMAGE

FROM ${BASE_IMAGE}

USER root

RUN apt-get update \
  && apt-get install -y \
    xorg-dev \
    libusb-1.0-0-dev \
    libxinerama-dev \
    python3 \
    python3-dev \
    libpython3.8-dev \
    gcc-8 g++-8 \
  && apt-get clean \
  && rm /usr/bin/gcc /usr/bin/g++ \
  && ln -s gcc-8 /usr/bin/gcc \
  && ln -s g++-8 /usr/bin/g++ \
  && apt-get clean

ENV REALSENSE_BASE=/home/root
ENV REALSENSE_DIR=$REALSENSE_BASE/librealsense

# clone librealsense SKD
RUN git clone https://github.com/IntelRealSense/librealsense.git $REALSENSE_DIR \
    && cd $REALSENSE_DIR \
    && git checkout b874e42 \
    && mkdir build

# compile librealsense SDK
RUN cd $REALSENSE_DIR/build \
  && sed  -i 's/if (CMAKE_VERSION VERSION_LESS 3.12)/if (CMAKE_VERSION VERSION_LESS 3.19)/g' ../wrappers/python/CMakeLists.txt \
  && cmake \
    -DCMAKE_BUILD_TYPE=release \
    -DBUILD_EXAMPLES=true \
    -DFORCE_RSUSB_BACKEND=ON \
    -DBUILD_WITH_CUDA=true \
    -DBUILD_PYTHON_BINDINGS=bool:true \
    -DPYBIND11_INSTALL=ON \
    -DPYTHON_EXECUTABLE:FILEPATH=$(python -c "import sys; print(sys.executable)") \
    -DPYTHON_INCLUDE_DIR:PATH=$(python -c "import sysconfig; print(sysconfig.get_path('include'))") \
    -DPYTHON_LIBRARY:FILEPATH=$(python -c "import sysconfig; import glob; print(glob.glob('/*/'.join(sysconfig.get_config_vars('LIBDIR', 'INSTSONAME')))[0])") \
    .. \
  && make -j`nproc` install

# Install realsense ROS 2 wrapper dependencies
RUN apt-get install python3-rosdep -y

# to make the "source" works
SHELL ["/bin/bash", "-c"]

# Install realsense ROS 2 wrapper
RUN mkdir -p /home/root/ros2_pre_installed/src \
  && cd /home/root/ros2_pre_installed/src \
  && git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development \
  && cd realsense-ros \
  && git checkout 6dcdc1fc0b898e38081e83edde8d5cea0e1e7c8b \
  && cd /home/root/ros2_pre_installed \
  && rosdep update \
  && rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y \
  && source /opt/ros/foxy/setup.bash \
  && colcon build \
    --packages-up-to realsense2_camera realsense2_camera_msgs realsense2_description

RUN curl https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules \
  -o /etc/udev/rules.d/99-realsense-libusb.rules
RUN echo '# Intel Realsense PYTHON PATH' >> /etc/bash.bashrc \
  && echo 'PYTHONPATH=$PYTHONPATH:'"$REALSENSE_DIR"'/usr/local/lib' >> /etc/bash.bashrc \
  && echo "source /home/root/ros2_pre_installed/install/setup.bash" >> /etc/bash.bashrc