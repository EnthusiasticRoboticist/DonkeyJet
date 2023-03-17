#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <chrono>
#include <errno.h>
#include <sys/ioctl.h>
#include "rclcpp/rclcpp.hpp"
#include "bot_hardware/msg/joy2.hpp"


class JoyStick : public rclcpp::Node
{
public:
  JoyStick() : Node("joystick")
  {
    std::string device{"/dev/input/js0"};
    js_fd_ = open(device.c_str(), O_RDONLY | O_NONBLOCK);
    while (js_fd_ < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open joystick device %d, trying again in 1 sec.", js_fd_);
      close(js_fd_);
      usleep(1000000);
      js_fd_ = open(device.c_str(), O_RDONLY | O_NONBLOCK);
    }

    number_of_axes = get_axis_count(js_fd_);
    number_of_buttons = get_button_count(js_fd_);

    RCLCPP_INFO(get_logger(), "joystick with %d axis and %d buttons is opened", number_of_axes, number_of_buttons);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1), std::bind(&JoyStick::timer_callback, this));
    
    js_publisher_ = this->create_publisher<bot_hardware::msg::Joy2>("/joy", 10);

    js_msg_.header.stamp = this->get_clock()->now();
    js_msg_.header.frame_id = "";
    
    // initialzie with bot_hardware::msg::Joy2::INVALID means it hasn't been updated from joystick yet
    js_msg_.button = bot_hardware::msg::Joy2::INVALID;
    js_msg_.button_value = bot_hardware::msg::Joy2::INVALID;
    js_msg_.axis = bot_hardware::msg::Joy2::INVALID;
    js_msg_.axis = bot_hardware::msg::Joy2::INVALID;
  }

private:
  void timer_callback()
  {
    struct js_event event;
    while (read_event(js_fd_, &event) == 0)
    {
      switch (event.type)
      {
      case JS_EVENT_BUTTON:
        js_msg_.button = event.number;
        js_msg_.button_value = event.value;
        js_msg_.axis = bot_hardware::msg::Joy2::INVALID;
        js_msg_.axis_value = bot_hardware::msg::Joy2::INVALID;
        RCLCPP_INFO(this->get_logger(), "Button %u %s", js_msg_.button, js_msg_.button_value ? "pressed" : "released");
        publish();
        break;
      case JS_EVENT_AXIS:
        js_msg_.button = bot_hardware::msg::Joy2::INVALID;
        js_msg_.button_value = bot_hardware::msg::Joy2::INVALID;
        js_msg_.axis = event.number;
        js_msg_.axis_value = event.value / 32767.0;
        RCLCPP_INFO(this->get_logger(), "Axis %u at %6f", js_msg_.axis, js_msg_.axis_value);
        publish();
        break;
      default:
        /* Ignore init events. */
        break;
      }

    }
  }

  void publish()
  {
    js_msg_.header.stamp = this->get_clock()->now();
    js_publisher_->publish(js_msg_);
  }

  /**
   * Reads a joystick event from the joystick device.
   * Returns 0 on success. Otherwise -1 is returned.
   */
  int read_event(int fd, struct js_event *event)
  {
    ssize_t bytes;

    bytes = read(fd, event, sizeof(*event));

    if (bytes == sizeof(*event))
      return 0;

    return errno;
  }

  /**
   * Returns the number of axes on the controller or 0 if an error occurs.
   */
  size_t get_axis_count(int fd)
  {
    __u8 axes;

    if (ioctl(fd, JSIOCGAXES, &axes) == -1)
      return 0;

    return axes;
  }

  /**
   * Returns the number of buttons on the controller or 0 if an error occurs.
   */
  size_t get_button_count(int fd)
  {
    __u8 buttons;
    if (ioctl(fd, JSIOCGBUTTONS, &buttons) == -1)
      return 0;

    return buttons;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  int js_fd_;
  unsigned int number_of_axes;
  unsigned int number_of_buttons;
  rclcpp::Publisher<bot_hardware::msg::Joy2>::SharedPtr js_publisher_;
  bot_hardware::msg::Joy2 js_msg_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyStick>());
  rclcpp::shutdown();
  return 0;
}