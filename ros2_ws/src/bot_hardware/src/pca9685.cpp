#include <memory>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include "rclcpp/rclcpp.hpp"
#include "bot_hardware/msg/joy2.hpp"

using std::placeholders::_1;

#define I2C_ADDRESS 0x40
#define PCA9685_MODE1 0x00
#define PCA9685_MODE2 0x01
#define PCA9685_LED0_ON_L 0x06
#define PCA9685_PRESCALE 0xFE

#define THROTTLE_CHANNEL 0
#define STEERING_CHANNEL 1

#define JOYSTICK_THROTTLE_AXIS 1
#define JOYSTICK_STEERING_AXIS 2

class Float2ServoMap
{
public:
  using Ptr = std::shared_ptr<Float2ServoMap>;

  static Float2ServoMap::Ptr make(float freq, float minMiliSec, float maxMiliSec, unsigned int maxPWM)
  {
    return std::make_shared<Float2ServoMap>(freq, minMiliSec, maxMiliSec, maxPWM);
  }

  float calPwm(float f)
  {
    return m_ * f + b_;
  }

  Float2ServoMap(float freq, float minMiliSec, float maxMiliSec, unsigned int maxPWM):
    m_(0), b_(0)
  {
    float ms_2_percent = float(freq) / 1000.0 /* ms */;
    float minPWMv = minMiliSec * ms_2_percent * maxPWM;
    float maxPWMv = maxMiliSec * ms_2_percent * maxPWM;
    float Fmin = -1.0;
    float Fmax = 1.0;
    m_ = (maxPWMv - minPWMv) / (Fmax - Fmin);
    b_ = minPWMv - m_ * Fmin;
    std::cout << "minPWMv " << minPWMv << " maxPWMv " << maxPWMv << std::endl;
    std::cout << "m_ " << m_ << " b_ " << b_ << std::endl;
  }
private:
  float m_, b_;
};

class PCA9685 : public rclcpp::Node
{
public:
  PCA9685() : Node("pca9685")
  {
    init_hardware();
    js_subscription_ = this->create_subscription<bot_hardware::msg::Joy2>(
        "/joy", 10, std::bind(&PCA9685::js_callback, this, _1));
  }

  ~PCA9685()
  {
    // Close the I2C device
    set_pwm(0, 0, 0);
    set_pwm(1, 0, 0);
    close(i2c_fd_);
  }

private:
  void init_hardware()
  {
    // Set the PWM frequency
    pwm_freq_ = 60.0;
    steeringMap = Float2ServoMap::make(pwm_freq_, 1.0, 2.0, 4095u);
    throttleMap = Float2ServoMap::make(pwm_freq_, 1.0, 2.0, 4095u);

    // Get the I2C device file descriptor
    i2c_fd_ = open("/dev/i2c-1", O_RDWR);
    while (i2c_fd_ < 0)
    {
      RCUTILS_LOG_ERROR_NAMED(get_name(), "Failed to open I2C device: %d, trying again in 1 sec.", i2c_fd_);
      usleep(1000000);
      i2c_fd_ = open("/dev/i2c-1", O_RDWR);
    }

    // Set the I2C slave address
    dev_address_ = I2C_ADDRESS;
    if (ioctl(i2c_fd_, I2C_SLAVE, dev_address_) < 0)
    {
      RCUTILS_LOG_ERROR_NAMED(get_name(), "Failed to set I2C slave address");
      return;
    }
    
    set_frequency(pwm_freq_);

    // Set the default PWM values
    set_pwm(0, 0, 0);
    set_pwm(1, 0, 0);
  }

  // Set the PWM frequency (in Hz)
  void set_frequency(uint16_t frequency)
  {
    uint8_t prescale = (uint8_t)(std::round(25000000.0 / (4096.0 * frequency)) - 1);
    write_reg(PCA9685_MODE1, 0b00010000); 
    usleep(1000);
    write_reg(PCA9685_PRESCALE, prescale);
    write_reg(PCA9685_MODE1, 0b10100000);
    usleep(1000);
    // write_reg(PCA9685_MODE2, 0x00);
    // usleep(1000);
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

  void write_reg(uint8_t reg, uint8_t data)
  {
    uint8_t buf[2] = {reg, data};
    if (write(i2c_fd_, buf, 2) != 2)
    {
      RCUTILS_LOG_ERROR_NAMED(get_name(), "Failed to write I2C register value");
    }
  }

  uint8_t read_reg(uint8_t reg)
  {
    uint8_t buf[] = {reg};
    if (write(i2c_fd_, buf, 1) != 2)
    {
      RCUTILS_LOG_ERROR_NAMED(get_name(), "Failed to write I2C register value");
    }
    if (read(i2c_fd_, buf, 1) != 2)
    {
      RCUTILS_LOG_ERROR_NAMED(get_name(), "Failed to read I2C register value");
    }
    return buf[0];
  }

  void js_callback(const bot_hardware::msg::Joy2::SharedPtr msg)
  {
    if (msg->axis == JOYSTICK_THROTTLE_AXIS)
    {
      setThrottle(msg->axis_value);
    } else if (msg->axis == JOYSTICK_STEERING_AXIS)
    {
      setSteering(msg->axis_value);
    }
  }

  // between -1 and 1 respectively full backware and full forward
  void setThrottle(float throttle)
  {
    throttle *= -1.0;
    if (std::abs(throttle) < 0.001 ) throttle = 0;
    uint16_t pwm = (uint16_t)(std::round(throttleMap->calPwm(throttle))) & 0x0FFF ;
    RCLCPP_DEBUG(this->get_logger(), "THROTTLE %d, RAW %f", pwm, throttle);
    set_pwm(THROTTLE_CHANNEL, 0, pwm);
  }

  // between -1 and 1 respectively
  void setSteering(float steering)
  {
    steering *= -1.0;
    if (std::abs(steering) < 0.0001 ) steering = 0;
    steering += 0.29;
    uint16_t pwm = (uint16_t)(std::round(steeringMap->calPwm(steering))) & 0x0FFF ;
    RCLCPP_DEBUG(this->get_logger(), "STEERING %d, RAW %f", pwm, steering);
    set_pwm(STEERING_CHANNEL, 0, pwm);
  }

  rclcpp::Subscription<bot_hardware::msg::Joy2>::SharedPtr js_subscription_;
  int i2c_fd_;
  uint8_t dev_address_;
  float pwm_freq_;
  Float2ServoMap::Ptr steeringMap;
  Float2ServoMap::Ptr throttleMap;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCA9685>());
  rclcpp::shutdown();
  return 0;
}