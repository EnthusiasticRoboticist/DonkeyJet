# Joystick (Controller)

[Read more](https://www.enthusiasticroboticist.com/blog/using-bluetooth-controller-with-ros-2-on-jetson-nano/)
[About Joystick in Linux](https://opencoursehub.cs.sfu.ca/bfraser/grav-cms/cmpt433/links/files/2022-student-howtos/LinuxJoystick.hLibrary.pdf).

### Test the joystick

With GUI
```bash
sudo apt install -y jstest-gtk
jstest-gtk
```

Without GUI
```bash
jstest /dev/input/js0
```

With ros
```
docker -H 192.168.68.68 pull ${REGISTRY}/ros2_base2:latest
docker -H 192.168.68.68 run \
  --rm -it  --runtime nvidia --network host \
  --gpus all --privileged \
  -v /dev:/dev -v /proc:/proc -v /sys:/sys \
  ${REGISTRY}/ros2_base2:latest \
  bash
```
