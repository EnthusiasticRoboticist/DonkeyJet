# ROS2 cheatsheat

```bash
source /opt/ros/foxy/setup.bash
ros2 pkg create bot_hardware --build-type ament_cmake --dependencies rclcpp
colcon build --symlink-install
source /ros2_ws/install/setup.bash
```