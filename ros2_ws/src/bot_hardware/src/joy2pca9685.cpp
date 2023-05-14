#include "bot_hardware/msg/control.hpp"
#include "bot_hardware/msg/joy2.hpp"
#include "rclcpp/rclcpp.hpp"

#define JOYSTICK_THROTTLE_AXIS 1
#define JOYSTICK_STEERING_AXIS 2

class Joy2Pca9685 : public rclcpp::Node {
public:
  Joy2Pca9685() : Node("joy2pca9685") {
    js_subscription_ = this->create_subscription<bot_hardware::msg::Joy2>(
        "/joy", 10,
        std::bind(&Joy2Pca9685::js_callback, this, std::placeholders::_1));
    control_publisher_ =
        this->create_publisher<bot_hardware::msg::Control>("/control", 10);
  }

private:
  void js_callback(const bot_hardware::msg::Joy2::SharedPtr js_msg) {
    bot_hardware::msg::Control control_msg;
    control_msg.header = js_msg->header;
    if (js_msg->axis == JOYSTICK_THROTTLE_AXIS) {
      control_msg.throttle = js_msg->axis_value;
      control_msg.steering = bot_hardware::msg::Control::INVALID;
      control_publisher_->publish(control_msg);
    } else if (js_msg->axis == JOYSTICK_STEERING_AXIS) {
      control_msg.throttle = bot_hardware::msg::Control::INVALID;
      control_msg.steering = js_msg->axis_value;
      control_publisher_->publish(control_msg);
    }
  }

  rclcpp::Subscription<bot_hardware::msg::Joy2>::SharedPtr js_subscription_;
  rclcpp::Publisher<bot_hardware::msg::Control>::SharedPtr control_publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Joy2Pca9685>());
  rclcpp::shutdown();
  return 0;
}