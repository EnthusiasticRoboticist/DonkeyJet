#include <iostream>
#include <memory>
#include <libevdev-1.0/libevdev/libevdev.h>
#include <fcntl.h>    /* For O_RDWR */
#include <unistd.h>   /* For open(), creat() */
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoystickNode : public rclcpp::Node
{
public:
  JoystickNode()
      : Node("joystick_node")
  {
    // Get the path to the joystick device
    std::string device_path = "/dev/input/js0"; // Change this to match your joystick device
    // std::string device_path = "/dev/input/event3"; // Change this to match your joystick device

    // Open the joystick device
    device_fd_ = open(device_path.c_str(), O_RDONLY | O_NONBLOCK);
    if (device_fd_ < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open joystick device %d", device_fd_);
      return;
    }

    device_ = libevdev_new();
    if (!device_)
        return;
      
    
    int err = libevdev_set_fd(device_, device_fd_);
    if (err < 0) {
        printf("Failed (errno %d): %s\n", -err, strerror(-err));
        return;
    }

    // Initialize libevdev for the device
    // int result = libevdev_new_from_fd(device_fd_, &device_);
    // if (result < 0)
    // {
    //   RCLCPP_ERROR(this->get_logger(), "Failed to initialize libevdev %d", result);
    //   close(device_fd_);
    //   return;
    // }

    // Set up the publisher for joystick events
    joy_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("joy", 10);

    // Set up the timer for reading joystick events
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&JoystickNode::readJoystickEvents, this));
  }

private:
  void readJoystickEvents()
  {
    // Read events from the joystick device
    while (true)
    {
      struct input_event ev;
      int result = libevdev_next_event(device_, LIBEVDEV_READ_FLAG_NORMAL, &ev);
      if (result == LIBEVDEV_READ_STATUS_SUCCESS)
      {
        switch (ev.type)
        {
        case EV_KEY:
          // Publish button events
          {
            sensor_msgs::msg::Joy joy_msg;
            joy_msg.header.stamp = this->now();
            joy_msg.buttons.resize(16);
            joy_msg.buttons[ev.code] = ev.value;
            joy_publisher_->publish(joy_msg);
          }
          break;

        case EV_ABS:
          // Publish axis events
          {
            sensor_msgs::msg::Joy joy_msg;
            joy_msg.header.stamp = this->now();
            joy_msg.axes.resize(8);
            switch (ev.code)
            {
            case ABS_X:
              joy_msg.axes[0] = ev.value / 32767.0;
              break;

            case ABS_Y:
              joy_msg.axes[1] = ev.value / 32767.0;
              break;

            case ABS_Z:
              joy_msg.axes[2] = ev.value / 32767.0;
              break;

            case ABS_RX:
              joy_msg.axes[3] = ev.value / 32767.0;
              break;

            case ABS_RY:
              joy_msg.axes[4] = ev.value / 32767.0;
              break;

            case ABS_RZ:
              joy_msg.axes[5] = ev.value / 32767.0;
              break;

            case ABS_HAT0X:
              joy_msg.axes[6] = ev.value;
              break;

            case ABS_HAT0Y:
              joy_msg.axes[7] = ev.value;
              break;

            default:
              break;
            }
            joy_publisher_->publish(joy_msg);
          }
          break;

        default:
          break;
        }
      }
      else if (result == LIBEVDEV_READ_STATUS_SYNC)
      {
        // Synchronization event, ignore it
      }
      else if (result == -EAGAIN)
      {
        // No more events available for now, exit the loop
        break;
      }
      else
      {
        // Error reading event, log and exit
        RCLCPP_ERROR(this->get_logger(), "Failed to read joystick event");
        return;
      }
    }
  }

  int device_fd_;
  struct libevdev *device_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoystickNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}