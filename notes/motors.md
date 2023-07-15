# motors

[Read more here](https://www.enthusiasticroboticist.com/blog/actuation-and-pca9685-with-ros-2-on-jetson-nano/)

```bash
docker run \
  --name realsense \
  --rm \
  -it \
  --privileged \
  --network host \
  -e DISPLAY \
  --runtime nvidia \
  --gpus all \
  ${REGISTRY}/ros2_realsense:latest \
  bash

  -v /dev/input:/dev/input \
  -v /dev/v4l:/dev/v4l \
  -v /dev:/dev \
  -v /proc:/proc \
  -v /sys:/sys \
# issue about Jetson.GPIO inside docker: 
# https://forums.developer.nvidia.com/t/jetson-gpio-not-working-on-python-inside-the-container/180435
python -m pip install adafruit-circuitpython-servokit Jetson.GPIO
i2cdetect -y -r 1
```

- [More about I2C in Linux](https://www.youtube.com/watch?v=-1PHQYRbAm8&ab_channel=Johannes4GNU_Linux)
- [PCA9685 datasheet](https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf)
- [PCA9685 Adafruit info](https://cdn-learn.adafruit.com/downloads/pdf/16-channel-pwm-servo-driver.pdf)
- The I2C address is `0x40`.
- [About Actutation](http://docs.donkeycar.com/parts/actuators/)
- [I2C in Linux](http://embeddedcraft.org/eclinux/linuxi2c.html)

## PWM Signals
- PWM Period: 20 ms, PWM Frequency: 50Hz
- 1ms - 2ms high or Duty cycle: 1% - 10% with 50Hz freq
- Servo from left to right
- Throttle form full backward to full forward.

```
freq = 60
prescaler = 0x66 = 25M / (4096 * frequency)
i2cset -y 1 0x40 0x00 0x90
i2cset -y 1 0x40 0xFE 0x66
i2cset -y 1 0x40 0x00 0xA0

# steering
i2cset -y 1 0x40 0x0A 0x00 && \
  i2cset -y 1 0x40 0x0B 0x00 && \
  i2cset -y 1 0x40 0x0C 0xFA && \
  i2cset -y 1 0x40 0x0D 0x00

i2cset -y 1 0x40 0x0A 0x00 0x00 0x4F 0x01 i

# throttle (85 instead of 6f is good value for test)
i2cset -y 1 0x40 0x06 0x00 && \
i2cset -y 1 0x40 0x07 0x00 && \
i2cset -y 1 0x40 0x08 0x85 && \
i2cset -y 1 0x40 0x09 0x01

i2cset -y 1 0x40 0x06 0x00 0x00 0x4F 0x01 i

```
