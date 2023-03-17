#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define PCA9685_MODE1 0x00
#define PCA9685_MODE2 0x01
#define PCA9685_LED0_ON_L 0x06
#define PCA9685_PRESCALE 0xFE

// Write a byte to an I2C register
void
write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    if (write(i2c_fd_, buf, 2) != 2)
    {
        RCUTILS_LOG_ERROR_NAMED(get_name(), "Failed to write I2C register value");
    }
}

class PCA9685Node : public rclcpp::Node
{
public:
    PCA9685Node() : Node("pca9685_i2c_node")
    {
        // Get the I2C device file descriptor
        i2c_fd_ = open("/dev/i2c-1", O_RDWR);
        if (i2c_fd_ < 0)
        {
            RCUTILS_LOG_ERROR_NAMED(get_name(), "Failed to open I2C device");
            return;
        }

        // Set the I2C slave address
        dev_address_ = 0x40;
        if (ioctl(i2c_fd_, I2C_SLAVE, dev_address_) < 0)
        {
            RCUTILS_LOG_ERROR_NAMED(get_name(), "Failed to set I2C slave address");
            return;
        }

        // Initialize the PCA9685 chip
        initialize();

        // Create a timer that will publish PWM signals
        pwm_timer_ = create_wall_timer(std::chrono::microseconds(500), [this]()
                                       {
      // Increment the duty cycle
      pwm_duty_cycle_ += 5;
      if (pwm_duty_cycle_ > 4095) {
        pwm_duty_cycle_ = 0;
      }

      // Set the PWM duty cycle for channel 0
      set_pwm(0, 0, pwm_duty_cycle_); });
    }

    ~PCA9685Node()
    {
        // Disable the timer
        pwm_timer_->cancel();

        // Close the I2C device
        close(i2c_fd_);
    }

private:
    int i2c_fd_;
    uint8_t dev_address_;

    rclcpp::TimerBase::SharedPtr pwm_timer_;
    uint16_t pwm_duty_cycle_ = 0;
    double tick_per_us_;

    // Initialize the PCA9685 chip
    void initialize()
    {
        // Set the PWM frequency
        set_frequency(60);

        // Set the default LED on/off values
        for (int i = 0; i < 16; ++i)
        {
            set_pwm(i, 0, 0);
        }

        // Enable the chip's auto-increment feature
        uint8_t mode1 = read_reg(PCA9685_MODE1);
        mode1 |= 0x20;
        write_reg(PCA9685_MODE1, mode1);
    }

    // Set the PWM frequency (in Hz)
    void set_frequency(uint16_t frequency)
    {
        double tick_resolution = 4096.0 / 25000000.0;
        tick_resolution *= frequency;
        tick_per_us_ = tick_resolution;

        uint8_t prescale = std::round(25000000.0 / (4096.0 * frequency));
        write_reg(PCA9685_MODE1, 0x10);
        write_reg(PCA9685_PRESCALE, prescale);
        write_reg(PCA9685_MODE1, 0x80);
        write_reg(PCA9685_MODE2, 0x04);
    }

    // Set the PWM duty cycle for a specific channel
    void set_pwm(uint8_t channel, uint16_t on_time, uint16_t off_time)
    {
        uint8_t reg_offset = PCA9685_LED0_ON_L + 4 * channel;
        uint8_t on_time_low = on_time & 0xFF;
        uint8_t on_time_high = (on_time >> 8) & 0x0F;
        uint8_t off_time_low = off_time & 0xFF;
        uint8_t off_time_high = (off_time >> 8) & 0x0F;

        write_reg(reg_offset, on_time_low);
        write_reg(reg_offset + 1, on_time_high);
        write_reg(reg_offset + 2, off_time_low);
        write_reg(reg_offset + 3, off_time_high);
    }

    uint8_t data;
    if (read(i2c_fd_, &data, 1) != 1)
    {
        RCUTILS_LOG_ERROR_NAMED(get_name(), "Failed to read I2C register value");
        return 0;
    }

    return data;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCA9685Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}