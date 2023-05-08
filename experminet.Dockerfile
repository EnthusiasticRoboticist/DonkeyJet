
# rtab-map
RUN mkdir -p /root/ros2_rtabmap_ws \
  && cd /root/ros2_rtabmap_ws \
  && git clone https://github.com/introlab/rtabmap.git src/rtabmap \
  && git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
RUN cd /root/ros2_rtabmap_ws \
  && source /etc/bash.bashrc \
  && rosinstall_generator pcl_ros | vcs import src \
  && rosdep update && rosdep install --from-paths src --ignore-src -r -y --rosdistro $ROS_DISTRO
RUN cd /root/ros2_rtabmap_ws \
  && source /etc/bash.bashrc \
  && source ${ROS_ROOT}/install/setup.bash \
  && export MAKEFLAGS="-j`nproc`" \
  && colcon build --symlink-install --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DRTABMAP_SYNC_USER_DATA=ON -DCMAKE_BUILD_TYPE=Release \
  && echo "source /root/ros2_rtabmap_ws/install/setup.bash" >> /etc/bash.bashrc