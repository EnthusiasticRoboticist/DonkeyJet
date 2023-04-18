#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <librealsense2/rs.hpp>

class RealSense : public rclcpp::Node
{
public:
  RealSense() : Node("realsense")
  {
    setup_realsense();


    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1), std::bind(&RealSense::timer_callback, this));
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  }

private:
  void setup_realsense()
  {
    rs2::device rs_device;
    // auto dev = query_rs_device();
    // RCLCPP_INFO(this->get_logger(),"Resetting device...");
    // dev.hardware_reset();
    // dev = rs2::device();
    rs_device = query_rs_device();

    // enable stream of IMU data
    rs_cfg_.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 63);
    rs_cfg_.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200);
    rs_pipe_.start(rs_cfg_);

    RCLCPP_INFO(this->get_logger(), "realsense setup is complete!");
  }

  rs2::device query_rs_device()
  {
    rs2::context ctx;
    auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
    if (list.size() == 0) 
        throw std::runtime_error("No device detected. Is it plugged in?");
    rs2::device dev = list.front();
    RCLCPP_INFO(this->get_logger(),"Device name: %s", dev.get_info(RS2_CAMERA_INFO_NAME));
    return dev;
  }

  void timer_callback()
  {
    rs2::frameset rs_frames;
    if (!rs_pipe_.poll_for_frames(&rs_frames))
    {
      // no frame is ready
      RCLCPP_DEBUG(this->get_logger(), "no frame ready");
      return;
    }

    // for (auto f : rs_frames)
    // {
    //   rs2::stream_profile profile = f.get_profile();
    //   unsigned long fnum = f.get_frame_number();
    //   double ts = f.get_timestamp();
    // }

    auto fa = rs_frames.first(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    rs2::motion_frame accel = fa.as<rs2::motion_frame>();
    if(accel)
    {
      rs2_vector values = accel.get_motion_data();
      RCLCPP_INFO(this->get_logger(), "accel: %f, %f, %f", values.x, values.y, values.z);
    }

    auto fg = rs_frames.first(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
    rs2::motion_frame gyro = fg.as<rs2::motion_frame>();
    if(gyro)
    {
      rs2_vector values = gyro.get_motion_data();
      RCLCPP_INFO(this->get_logger(), "gyro: %f, %f, %f", values.x, values.y, values.z);
    }
  }

  void publish()
  {
    odom_msg_.header.stamp = this->get_clock()->now();
    odom_publisher_->publish(odom_msg_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  nav_msgs::msg::Odometry odom_msg_;
  rs2::pipeline rs_pipe_;
  rs2::config rs_cfg_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealSense>());
  rclcpp::shutdown();
  return 0;
}