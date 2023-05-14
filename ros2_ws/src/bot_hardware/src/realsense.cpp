#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rs_wrapper_lib.hpp"
#include <librealsense2/rs.hpp>

class RealSense : public rclcpp::Node {
public:
  RealSense() : Node("realsense") {
    setup_realsense_sensors();

    odom_publisher_ =
        this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  }

private:
  void setup_realsense_sensors() {
    rs2::context ctx;

    for (auto dev : ctx.query_devices()) {

      std::cout << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;

      if (strcmp(dev.get_info(RS2_CAMERA_INFO_NAME), "Intel RealSense D435") ==
          0) {

        auto sensor = donkeyjet::hardware::D435::create(dev);
        sensors_.push_back(sensor);

      } else if (strcmp(dev.get_info(RS2_CAMERA_INFO_NAME),
                        "Intel RealSense T265") == 0) {

        auto sensor = donkeyjet::hardware::T265::create(dev);
        sensor->callback = [](const rs2::frame &frame) {
          if (frame.is<rs2::motion_frame>()) {
            std::cout << "IMU data in new callback" << std::endl;
          } else if (frame.is<rs2::pose_frame>()) {
            std::cout << "Pose data in new callback" << std::endl;
          }
        };
        sensors_.push_back(sensor);
      }
    }

    for (auto &sensor : sensors_) {
      sensor->start_stream();
    }
  }

  void publish() {
    odom_msg_.header.stamp = this->get_clock()->now();
    odom_publisher_->publish(odom_msg_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  nav_msgs::msg::Odometry odom_msg_;
  rs2::pipeline rs_pipe_;
  rs2::config rs_cfg_;
  std::vector<donkeyjet::hardware::RealsenseSensorPtr> sensors_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealSense>());
  rclcpp::shutdown();
  return 0;
}